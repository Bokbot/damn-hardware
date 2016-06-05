/*
   Copyright (c) 2015 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read accelerometer data
*/

#include "CurieIMU.h"
//#include <DateTime.h>

/*#include <Time.h>*/
/*#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by unix time_t as ten ascii digits*/
/*#define TIME_HEADER  'T'   // Header tag for serial time sync message*/
/*#define TIME_REQUEST  7    // ASCII bell character requests a time sync message */

int oldHeartRate = 75;  // last heart rate reading from analog input
int playerID = 110;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms
unsigned long timeSeed = 1465135837;
unsigned long countZero = 1465135837;
unsigned long now1;


void updateHeartRate() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */
  int heartRateMeasurement = analogRead(A0);
  int heartRate = map(heartRateMeasurement, 0, 1023, 0, 100);
  if (heartRate != oldHeartRate) {      // if the heart rate has changed
 //   Serial.print("Heart Rate is now: "); // print it
    //Serial.println(heartRate);
//    const unsigned char heartRateCharArray[2] = { 0, (char)heartRate };
 //   heartRateChar.setValue(heartRateCharArray, 2);  // and update the heart rate measurement characteristic
    if(heartRate > 60 && heartRate < 180){
      if(heartRate > oldHeartRate){

        oldHeartRate = oldHeartRate + 1;           // save the level for next comparison
      }
      if(heartRate < oldHeartRate){
        oldHeartRate = oldHeartRate - 1;           // save the level for next comparison

      }
      if(heartRate == oldHeartRate){
        oldHeartRate = heartRate;           // save the level for next comparison
      }
    }
  }
}

void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  //setTime(timeSeed);

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
}

void loop() {
  int axRaw, ayRaw, azRaw;         // raw accelerometer values
  float ax, ay, az;
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;

  // read raw gyro measurements from device
  CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  now1 = millis();

  // read raw accelerometer measurements from device
  CurieIMU.readAccelerometer(axRaw, ayRaw, azRaw);

  // convert the raw accelerometer data to G's
  ax = convertRawAcceleration(axRaw);
  ay = convertRawAcceleration(ayRaw);
  az = convertRawAcceleration(azRaw);

    long currentMillis = millis();
    while (currentMillis - previousMillis >= 200) {
      currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }
  // display tab-separated accelerometer x/y/z values
    Serial.print(playerID);
    Serial.print(',');
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(',');
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(oldHeartRate);
    Serial.print(",");
    //Serial.print(DateTime.now());
    Serial.print(countZero);
    Serial.print(",");
    Serial.println(millis());
    countZero++;

  // wait 5 seconds
  delay(500);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;

  return a;
}


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;

}

