#include <QTRSensors.h>
#include <Arduino.h>


// Motor Variables
const int PWMA = 10;
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMB = 11;
const int BIN1 = A1;
const int BIN2 = A2;
const int STBY = A0;
const int motorSpeed = 100;
const int maxMotorSpeed = 255;


// PID Variables
double KP = 0.03;
double KI = 0.00;
double KD = 0.00;
int error = 0;
int previousError = 0;
int sumOfErrors = 0;


// Sensor declaration
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// Calibrate Sensors Function
 void calibrateLinesensor(){
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 500; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
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
void PIDcontrol(){
  sumOfErrors = error + previousError;
  int adjustedSpeed = (int)((KP * error) + (KD * (error - previousError)) + (KI * sumOfErrors));
  previousError = error;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, constrain(motorSpeed+adjustedSpeed,0,maxMotorSpeed));
  analogWrite(PWMB, constrain(motorSpeed-adjustedSpeed,0,maxMotorSpeed));

}



void setup()
{
  // Configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 12}, SensorCount);
  qtr.setEmitterPin(2);

  // Calibrate Sensors
  calibrateLinesensor();

  // Set pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Initialize motors
  digitalWrite(STBY, HIGH);  // Disable standby
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  // PID Control
  PIDcontrol();

  delay(10);
}
