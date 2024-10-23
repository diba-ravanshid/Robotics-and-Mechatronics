#include "I2Cdev.h"

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);



long timer = 0;



// ================================================================
//                          INITIAL SETUP                       
// ================================================================


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

// ================================================================
//                        MAIN PROGRAM LOOP                       
// ================================================================
unsigned long time = 0;
void loop() {
    mpu6050.update();
    double dt = 0.01;
    time = micros();
    float acc = mpu6050.getAccX();
    float gyro = mpu6050.getGyroX();
    float angle = mpu6050.getAngleX();
    float angle_new = 0.96 *(angle + gyro*dt) + 0.04*acc;
    Serial.print(angle);
    Serial.print(",");
    Serial.print(angle_new);
    Serial.println("\n");

}

