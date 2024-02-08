#include <QTRSensors.h>

QTRSensors qtr;

#define Kp  4.5
// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Ki 0.0
#define Kd  0.0
#define rightMaxSpeed 255 // max speed of the robot                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
#define leftMaxSpeed 255// max speed of the robot
#define rightBaseSpeed 250 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 250
int lastError = 0;
int I;
int pos=0;
const uint8_t SensorCount =6;
uint16_t sensorValues[SensorCount];
int t=1;
int yt=0;
const int encoderPin1 = 18;
const int encoderPin2 = 19;
const int encoderPin3 = 20;
const int encoderPin4 = 21;
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
//
#define rightenable 13 //enableB
#define leftenable 7 //enableA
#define rightMotor1 9 //int3
#define rightMotor2 10 //int4
#define leftMotor1 11 //int1
#define leftMotor2 12 //int2

void setup()
{
   // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);
   delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
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
  delay(1000);
 pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  //pinMode(rightenable, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
//  pinMode(leftenable, OUTPUT);

  Serial.begin(9600);
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
 Serial.print('\n');
  int pos = -(3*sensorValues[0]+2*sensorValues[1]+sensorValues[2]-sensorValues[3]-2*sensorValues[4]-3*sensorValues[5]);
//  Serial.println(pos);
  int error = pos + 22;
    I = I + error ;
    int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (I);
    lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    if (rightMotorSpeed > rightMaxSpeed ){
        rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    }
    if (leftMotorSpeed > leftMaxSpeed ){
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    }
    if (rightMotorSpeed < -rightMaxSpeed){
      rightMotorSpeed = 0; // keep the motor speed positive
    }
    if (leftMotorSpeed < -leftMaxSpeed){
      leftMotorSpeed = 0; // keep the motor speed positive
    }
    analogWrite(rightMotor2, 0);
      analogWrite(rightMotor1, rightMotorSpeed);
      analogWrite(leftMotor1, leftMotorSpeed);
      analogWrite(leftMotor2, 0);
//        Serial.println(rightMotorSpeed);
//    Serial.println(leftMotorSpeed);
//    digitalWrite(rightenable, HIGH);
//    digitalWrite(leftenable, HIGH);// move forward with appropriate speeds
//    if(sensorValues[0]>700 && sensorValues[1]>700 && sensorValues[2]>700 && sensorValues[3]>700 && sensorValues[4]>700 && sensorValues[5]>700 && sensorValues[6]>700 && sensorValues[7]>700 && sensorValues[8]>700 && sensorValues[9]>700 ){
////      if(x=0){
////          analogWrite(rightMotor1, 250);
////          analogWrite(rightMotor2, 0);
////          analogWrite(leftMotor1, 250);
////          analogWrite(leftMotor2, 0);
////      }
//      analogWrite(rightMotor1, 250);
//      analogWrite(rightMotor2, 0);
//      analogWrite(leftMotor1, 0);
//      analogWrite(leftMotor2, 100);  
//    }else if(sensorValues[2]<200 && sensorValues[3]<200 && sensorValues[4]<200 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200 && sensorValues[8]<200 && sensorValues[9]<200 ){
//      analogWrite(rightMotor1, 0);
//      analogWrite(rightMotor2, 100);
//      analogWrite(leftMotor1, 250);
//      analogWrite(leftMotor2, 0);
//      if(t>0){
//        yt=encoderValue1;
//        t=0; 
//      }else{
//        if(encoderValue1>yt+500){
//        if(sensorValues[0]<200 && sensorValues[1]<200 && sensorValues[2]<200 && sensorValues[3]<200 && sensorValues[4]<200 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200){
//          analogWrite(50,150);
//          analogWrite(52,0);
//          analogWrite(rightMotor1, 0);
//          analogWrite(rightMotor2, 0);
//          analogWrite(leftMotor1, 0);
//          analogWrite(leftMotor2, 0);
//          delay(10000);
//        }else{
//        t=1;
//        }
//      }
//      }
//    }else{
//      if (leftMotorSpeed>0){
//        analogWrite(rightMotor1, 0);
//      analogWrite(rightMotor2, leftMotorSpeed);
//      }else{
//        analogWrite(rightMotor1, -leftMotorSpeed);
//      analogWrite(rightMotor2, 0);
//      }
//      if (rightMotorSpeed>0){
//        analogWrite(leftMotor1, 0);
//      analogWrite(leftMotor2, rightMotorSpeed);
//      }else{
//        analogWrite(leftMotor1, -rightMotorSpeed);
//      analogWrite(leftMotor2, 0);
//      }
//    }
      


  //delay(250);
}
//void updateEncoder() {
//  int MSB1 = digitalRead(encoderPin1);
//  int LSB1 = digitalRead(encoderPin2);
//  
//  int encoded1 = (MSB1 << 1) | LSB1;
//  int sum1 = (lastEncoded1 << 2) | encoded1;
//
//  if (sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011)
//    encoderValue1--;
//  if (sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000)
//    encoderValue1++;
//
//  lastEncoded1 = encoded1;
//}
//
//void updateEncoder2() {
//  int MSB2 = digitalRead(encoderPin3);
//  int LSB2 = digitalRead(encoderPin4);
//  int encoded2 = (MSB2 << 1) | LSB2;
//  int sum2 = (lastEncoded2 << 2) | encoded2;
//
//  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011)
//    encoderValue2--;
//  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000)
//    encoderValue2++;
//
//  lastEncoded2 = encoded2;
//}