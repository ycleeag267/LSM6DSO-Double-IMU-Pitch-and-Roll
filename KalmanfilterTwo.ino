/******************************************************************************
Basic_Readings.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
Most basic example of use.

Example using the LSM6DSO with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  29 December 2022 -  initial release
*/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "SPI.h"

LSM6DSO SensorOne;

LSM6DSO SensorTwo;

float RateRoll1, RatePitch1, RateYaw1;
float RateRoll2, RatePitch2, RateYaw2;
float RateCalibrationRoll1, RateCalibrationPitch1, RateCalibrationYaw1;
float RateCalibrationRoll2, RateCalibrationPitch2, RateCalibrationYaw2;
int RateCalibrationNumber;
float AccX1, AccY1, AccZ1;
float AccX2, AccY2, AccZ2;
float AngleRoll1, AnglePitch1;
float AngleRoll2, AnglePitch2;
uint32_t LoopTimer;
float KalmanAngleRoll1=0, KalmanUncertaintyAngleRoll1=4;
float KalmanAnglePitch1=0, KalmanUncertaintyAnglePitch1=4;

float KalmanAngleRoll2=0, KalmanUncertaintyAngleRoll2=4;
float KalmanAnglePitch2=0, KalmanUncertaintyAnglePitch2=4;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.016*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.008 * 0.008 * 1 * 1;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {
  RateRoll1= SensorOne.readFloatGyroX();
  RatePitch1= SensorOne.readFloatGyroY();
  RateYaw1= SensorOne.readFloatGyroZ();
  AccX1= SensorOne.readFloatAccelX();
  AccY1= SensorOne.readFloatAccelY();
  AccZ1= SensorOne.readFloatAccelZ();
  AngleRoll1=atan(AccY1/sqrt(AccX1*AccX1+AccZ1*AccZ1))*1/(3.142/180);
  AnglePitch1=-atan(AccX1/sqrt(AccY1*AccY1+AccZ1*AccZ1))*1/(3.142/180);
//again for IMU 2
  RateRoll2= SensorTwo.readFloatGyroX();
  RatePitch2= SensorTwo.readFloatGyroY();
  RateYaw2= SensorTwo.readFloatGyroZ();
  AccX2= SensorTwo.readFloatAccelX();
  AccY2= SensorTwo.readFloatAccelY();
  AccZ2= SensorTwo.readFloatAccelZ();
  AngleRoll2=atan(AccY2/sqrt(AccX2*AccX2+AccZ2*AccZ2))*1/(3.142/180);
  AnglePitch2=-atan(AccX2/sqrt(AccY2*AccY2+AccZ2*AccZ2))*1/(3.142/180);
}
void setup() {
  Serial.begin(115200);
  delay(500); //relax...
  
  Wire.begin();
  delay(10);
  if( SensorOne.begin(106) )
    Serial.println("Ready1.");
  else { 
    Serial.println("Could not connect to IMU 1.");
    Serial.println("Freezing");
  }
   if( SensorTwo.begin(107) )
    Serial.println("Ready2.");
  else { 
    Serial.println("Could not connect to IMU 2.");
    Serial.println("Freezing");
  }
   if( SensorOne.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings1.");
   if( SensorTwo.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings2.");

  // for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
  //   gyro_signals();
  //   RateCalibrationRoll1+=RateRoll1;
  //   RateCalibrationPitch1+=RatePitch1;
  //   RateCalibrationYaw1+=RateYaw1;
    
  //   RateCalibrationRoll2+=RateRoll2;
  //   RateCalibrationPitch2+=RatePitch2;
  //   RateCalibrationYaw2+=RateYaw2;
  //   delay(2);
  // }
  // RateCalibrationRoll1/=2000;
  // RateCalibrationPitch1/=2000;
  // RateCalibrationYaw1/=2000;

  // RateCalibrationRoll2/=2000;
  // RateCalibrationPitch2/=2000;
  // RateCalibrationYaw2/=2000;
  // LoopTimer=micros();

  RateCalibrationRoll1=0;
  RateCalibrationPitch1=0;
  RateCalibrationYaw1=0;

  RateCalibrationRoll2=0;
  RateCalibrationPitch2=0;
  RateCalibrationYaw2=0;
  LoopTimer=micros();
}
void loop() {
  gyro_signals();
  RateRoll1-=RateCalibrationRoll1;
  RatePitch1-=RateCalibrationPitch1;
  RateYaw1-=RateCalibrationYaw1;

  RateRoll2-=RateCalibrationRoll2;
  RatePitch2-=RateCalibrationPitch2;
  RateYaw2-=RateCalibrationYaw2;

// compute angles
  kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll1);
  KalmanAngleRoll1=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll1=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch1);
  KalmanAnglePitch1=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch1=Kalman1DOutput[1];

  kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
  KalmanAngleRoll2=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll2=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
  KalmanAnglePitch2=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch2=Kalman1DOutput[1];

//these are the roll and pitch angles. pitch angles are relevant in the finger motion 
  Serial.print("Roll1:");
  Serial.print(KalmanAngleRoll1, 3);
  Serial.print(",");
  Serial.print("Pitch1:");
  Serial.print(KalmanAnglePitch1, 3);
  Serial.print(",");
  Serial.print("Roll2:");
  Serial.print(KalmanAngleRoll2, 3);
  Serial.print(",");
  Serial.print("Pitch2:");
  Serial.println(KalmanAnglePitch2, 3);


  while (micros() - LoopTimer < 8000);
    LoopTimer=micros();
}