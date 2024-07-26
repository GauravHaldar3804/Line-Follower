#include <QTRSensors.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WiFiCredentials.h>


// Motor Variables
const int PWMA = 12;
const int AIN1 = 13;
const int AIN2 = 21;
const int PWMB = 22;
const int BIN1 = 23;
const int BIN2 = 5;
const int STBY = 15;
const int motorSpeed = 100;
const int maxMotorSpeed = 255;

int count = 0;

// PID Variables
int KP = 0;
int KI = 0;
int KD = 0;
int error = 0;
int previousError = 0;
int sumOfErrors = 0;


// Sensor declaration
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Initilize Websocket
WebServer server(80);
WebSocketsServer websocket = WebSocketsServer(81);
JsonDocument doc_tx;
JsonDocument doc_rx;
bool system_started = false;

// Websocket HTML

String webpage = R"(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PID Controller</title>
  <style>
    body {
      font-family: sans-serif;
      text-align: center;
    }
    h1 {
      margin-bottom: 20px;
    }
    .controls {
      display: inline-block;
      margin: 0 10px;
    }
    .slider {
      width: 200px;
    }
    #values {
      margin-top: 20px;
      font-weight: bold;
    }
    button {
      padding: 10px 20px;
      margin: 10px;
      font-size: 16px;
    }
  </style>
</head>
<body>
  <h1>PID Controller</h1>
  <div class="controls">
    <h2>KP:</h2>
    <input type="range" min="0" max="100" step="0.1" value="0" id="kp-slider" class="slider">
    <p id="kp-value">0.0</p>
  </div>
  <div class="controls">
    <h2>KI:</h2>
    <input type="range" min="0" max="100" step="0.1" value="0" id="ki-slider" class="slider">
    <p id="ki-value">0.0</p>
  </div>
  <div class="controls">
    <h2>KD:</h2>
    <input type="range" min="0" max="100" step="0.1" value="0" id="kd-slider" class="slider">
    <p id="kd-value">0.0</p>
  </div>
  <div id="values">
    <button id="start-button">Start</button>
    <button id="stop-button">Stop</button>
  </div>
  <script>
    var Socket;

    function init() {
      Socket = new WebSocket("ws://" + window.location.hostname + ":81/");
      Socket.onmessage = function(event) {
        processCommand(event);
      };
    }

    function sendMessage(type, value) {
      var msg = {
        type: type,
        value: value
      };
      Socket.send(JSON.stringify(msg));
    }

    var kpSlider = document.getElementById('kp-slider');
    var kpValue = document.getElementById('kp-value');
    var kiSlider = document.getElementById('ki-slider');
    var kiValue = document.getElementById('ki-value');
    var kdSlider = document.getElementById('kd-slider');
    var kdValue = document.getElementById('kd-value');

    kpSlider.oninput = function() {
      kpValue.textContent = this.value;
      sendMessage("KP", this.value);
    };
    kiSlider.oninput = function() {
      kiValue.textContent = this.value;
      sendMessage("KI", this.value);
    };
    kdSlider.oninput = function() {
      kdValue.textContent = this.value;
      sendMessage("KD", this.value);
    };

    document.getElementById('start-button').addEventListener('click', function() {
      sendMessage("Start", "true");
    });

    document.getElementById('stop-button').addEventListener('click', function() {
      sendMessage("Stop", "false");
    });

    function processCommand(event) {
      var obj = JSON.parse(event.data);
      var type = obj.type;
      var value = obj.value;

      if (type === "KP") {
        kpSlider.value = value;
        kpValue.textContent = value;
      } else if (type === "KI") {
        kiSlider.value = value;
        kiValue.textContent = value;
      } else if (type === "KD") {
        kdSlider.value = value;
        kdValue.textContent = value;
      } else if (type === "Start") {
        system_started = value === "true";
      } else if (type === "Stop") {
        system_started = value === "false";
      }
    }

    window.onload = function(event) {
      init();
    };
  </script>
</body>
</html>
)";

// Sending message back to websocket
void sendJSON(const String& type, const String& value) {
  String jsonstring;
  JsonObject object = doc_tx.to<JsonObject>();
  object["type"] = type;
  object["value"] = value;
  serializeJson(doc_tx, jsonstring);
  websocket.broadcastTXT(jsonstring);
}


// Websocket Function

void onWSevent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("Client Connected");
      break;
    case WStype_DISCONNECTED:
      Serial.println("Client Disconnected");
      break;
    case WStype_TEXT:
      DeserializationError error = deserializeJson(doc_rx, payload);
      if (error) {
        Serial.print("Deserialization error: ");
        Serial.println(error.c_str());
        return;
      }
      const char* type = doc_rx["type"];
      const char* value = doc_rx["value"];

      Serial.print("Received: ");
      Serial.print("Type: ");
      Serial.print(type);
      Serial.print(" Value: ");
      Serial.println(value);

      if (strcmp(type, "KP") == 0) {
        KP = atof(value);
      } else if (strcmp(type, "KI") == 0) {
        KI = atof(value);
      } else if (strcmp(type, "KD") == 0) {
        KD = atof(value);
      } else if (strcmp(type, "Start") == 0) {
        system_started = true;
      } else if (strcmp(type, "Stop") == 0) {
        system_started = false;
      }

      sendJSON(type, value);
      break;
  }
}


// Websocket Setup Function
void websocketSetup(){
    WiFi.begin(SSID, Password);
  Serial.println("Establishing connection with " + String(SSID));

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi network with IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", [] { server.send(200, "text/html", webpage); });
  server.begin();
  websocket.begin();
  websocket.onEvent(onWSevent);
}

// Calibrate Sensors Function
 void calibrateLinesensor(){
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
 }



// PID Control Functions
void PIDcontrol(int Kp,int Ki,int Kd){
  Kp = KP * 0.001;
  Ki = KI * 0.001;
  Kd = Kd * 0.001;


  sumOfErrors = error + previousError;
  int adjustedSpeed = (int)((Kp * error) + (Kd * (error - previousError)) + (Ki * sumOfErrors));
  previousError = error;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, constrain(motorSpeed+adjustedSpeed,0,maxMotorSpeed));
  analogWrite(PWMB, constrain(motorSpeed-adjustedSpeed,0,maxMotorSpeed));

}


// Wait for Command
// void waitForSystemStart() {
//   Serial.println("Waiting for system to start...");
//   while (!system_started) {
  
//     // websocket.loop();
//     delay(100);  // Small delay to prevent busy waiting
//   }
//   Serial.println("System started!");
// }


void setup()
{

  Serial.begin(9600);

  // Websocket Setup
  websocketSetup();

  
  
  // Configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 18, 19}, SensorCount);
  qtr.setEmitterPin(4);


    // // Set pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // // Initialize motors
  digitalWrite(STBY, HIGH);




  // Disable standby
 delay(1000);
}

void loop()
{

  server.handleClient();
  websocket.loop();

    if (system_started== true){
// Calibrate Sensors
count++;
if(count==1){
  calibrateLinesensor();
  }
  

else if (count>1){
  // // read calibrated sensor values and obtain a measure of the line position
  // // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // // reflectance and 1000 means minimum reflectance, followed by the line
  // // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  // // PID Control
  PIDcontrol(KP,KI,KD);
}    
}

}