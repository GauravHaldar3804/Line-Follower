#include <QTRSensors.h>
#include <Arduino.h>
#include <SparkFun_TB6612.h>

// // Motor Variables
const int PWMA = 9;
const int AIN1 = A1;
const int AIN2 = A0;
const int PWMB = 10;
const int BIN1 = A4;
const int BIN2 = A5;
const int STBY = A3;
int motorSpeed = 70;
int maxMotorSpeed = 255;


// PID Variable?₹
float Kp = 0.087;
float Ki = 0;
float Kd = 0.25;
int error = 0;
int setpoint = 3500;
int previousError = 0;
int sumOfErrors = 0;
bool PID = true;

// Sensor declaration
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN2, AIN1, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);



// Calibrate Sensors Function
 void calibrateLinesensor(){
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode


   motor1.drive(100);
   motor2.drive(-100);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
 
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();

  }
  digitalWrite(LED_BUILTIN, LOW);
  brake(motor1, motor2); // turn off Arduino's LED to indicate we are through with calibration

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
void PIDcontrol(){

   uint16_t position = qtr.readLineBlack(sensorValues);
  error = position - setpoint;

   if(sensorValues[0]<=200 && sensorValues[1]<=200 && sensorValues[2]<=200 && sensorValues[3]<=200 && sensorValues[4]<=200 && sensorValues[5]<=200 && sensorValues[6]<=200 && sensorValues[7]<=200){
      PID = false;
   } // A case when the line follower leaves the line
    else {
    PID = true;
 }
   if(PID == false){
    if(previousError>0){ 
            //Turn left if the line was to the left before
   Serial.print("LEFT");
   Serial.print('\t');
      motor1.drive(125);
      motor2.drive(-125);
    }
    else{
   Serial.print("RIGHT");
   Serial.print('\t');
      motor1.drive(-125);
      motor2.drive(125);
       // Else turn right
    }
  }

  if (PID == true)
  {
     sumOfErrors = error + previousError;
     float adjustedSpeed = (Kp * error) + (Kd * (error - previousError)) + (Ki * sumOfErrors);
     previousError = error;

     int speedA = constrain(motorSpeed + adjustedSpeed, 0, maxMotorSpeed);
     int speedB = constrain(motorSpeed - adjustedSpeed, 0, maxMotorSpeed);

     motor1.drive(speedA);
     motor2.drive(speedB);

   //   Serial.print(speedA);
   //   Serial.print('\t');
   //   Serial.print(speedB);
   //   Serial.print('\t');
   //   Serial.print(error);
   //   Serial.println("");

  }
   Serial.print(PID);
   Serial.print('\t');
   Serial.print(previousError);
   Serial.println("");
}



void setup()
{

  Serial.begin(9600);
  // Configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2,3,4,5,6,7,8,11}, SensorCount);
  qtr.setEmitterPin(12);

  delay(1000);
  calibrateLinesensor();

 for (int i = 0; i < 3; i++)
 {
  digitalWrite(LED_BUILTIN, LOW);
 delay(500);
 digitalWrite(LED_BUILTIN, HIGH);
 delay(500);
 }
 digitalWrite(LED_BUILTIN, LOW);
 

}

void loop(){

  // // read calibrated sensor values and obtain a measure of the line position
  // // from 0 to 5000 (for a white line, use readLineWhite() instead)
  // // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // // reflectance and 1000 means minimum reflectance, followed by the line
  // // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  // Serial.println(position);

  // // PID Control
  PIDcontrol();
  // Serial.print("Error:");
  // Serial.println(error);
}

