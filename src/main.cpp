#include <Arduino.h>
#include <Wire.h>
#include <Orientation.h>
#include <I2Cdev.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>

MPU6050 mpu(Wire);
// Orientation vars
uint64_t thisLoopMicros = 0; // Stores microsecond timestamp for current loop
uint64_t lastOriUpdate = 0;  // Stores mircosecond timestamp for last time orientation updated
Orientation ori;             // Main orientation measurement
EulerAngles oriMeasure;      // Quaternion converted to Euler Angles for maths etc.
int cycles;
// Servo vars
Servo servoX;
Servo servoY;
int servoXpin = 3;
int servoYpin = 4;
int servoCount = 0;
float mpuPitch;
float mpuYaw;
// State Machine Vars
//  Pitch Yaw Count
int pitchYawToServoCount = 0;
enum flightStates
{
  IDLE,
  TVC,
  UNPOWERED_FLIGHT,
  DESCENT,
  LANDED
};

void setOrientation() {
  float mpuPitch;
  float mpuYaw;

   thisLoopMicros = micros(); // Get new microsecond timestamp for this loop
  mpu.update();
  float dtOri = (float)(thisLoopMicros - lastOriUpdate) / 1000000.; //  Finds elapsed mircroseconds since last update, converts to float, and converts to seconds
  lastOriUpdate = thisLoopMicros;    
  
  ori.update(mpu.getGyroZ() * DEG_TO_RAD, mpu.getGyroX() * DEG_TO_RAD, mpu.getGyroY() * DEG_TO_RAD, dtOri); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
  oriMeasure = ori.toEuler();  
   // Example use
  Serial.print("Yaw: ");
  Serial.print(oriMeasure.yaw * RAD_TO_DEG);                          //  We have updated, set the new timestamp

  //get Orientation Yaw and set to mpuPitchVar
  mpuPitch = oriMeasure.pitch;
  mpuYaw = oriMeasure.yaw;

}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  mpu.begin();
  mpu.calcGyroOffsets(true);
  thisLoopMicros = lastOriUpdate = micros(); // Set Starting time after init/calibration

  servoX.attach(servoXpin);
  servoY.attach(servoYpin);
}

void writePitchYawToServo();

void loop()
{
  writePitchYaw();
  delay(4);
}

void writePitchYaw() {
   thisLoopMicros = micros(); // Get new microsecond timestamp for this loop
  mpu.update();
  float dtOri = (float)(thisLoopMicros - lastOriUpdate) / 1000000.; //  Finds elapsed mircroseconds since last update, converts to float, and converts to seconds
  lastOriUpdate = thisLoopMicros;                                   //  We have updated, set the new timestamp

  /*
        This is where the magic actually happens
        
        The order of your axis measurements (x, y, z) will depend on your sensor, your reference frame, and your IMU library of choice
        Swap & invert your gyro measurements so that .update() is called with (yaw, pitch, roll, dt) in that order
        
        
        All gyro measurements must be measured right-handed (positive = yaw left, pitch down, roll right) and coverted to radians/sec
        */

  ori.update(mpu.getGyroZ() * DEG_TO_RAD, mpu.getGyroX() * DEG_TO_RAD, mpu.getGyroY() * DEG_TO_RAD, dtOri); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
  oriMeasure = ori.toEuler();

  /*
    Orientation measurement can then be used as follows:
    ori.orientation  : Main quaternion storing current orientation
    oriMeasure.yaw   
    oriMeasure.pitch 
    oriMeasure.roll  : Euler angles converted from quaternion (radians)
    */

  // Example use
  Serial.print("Yaw: ");
  Serial.print(oriMeasure.yaw * RAD_TO_DEG);
  Serial.print(", Pitch: ");
  Serial.print(oriMeasure.pitch * RAD_TO_DEG);
  Serial.println();
}

void setPitchYawToServos() {
mpuPitch = -oriMeasure.pitch * RAD_TO_DEG;
  mpuYaw = -oriMeasure.yaw * RAD_TO_DEG;
  // Limits servos to 10 degrees of freedom
  if (mpuPitch > -10 && mpuPitch < 10)
  {
    servoX.write(mpuPitch + 90);
  }
  if (mpuYaw > -10 && mpuYaw < 10) 
  {
    servoY.write(mpuYaw + 90);
  }
  
}