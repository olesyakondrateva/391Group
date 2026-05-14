/*
  Arduino BMI270 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270
  sensor, calculates the gyro angle, the time stamp, 
  and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  Reference code by Riccardo Rizzo

  */

#include "Arduino_BMI270_BMM150.h"

float gyro_angle = 0.0;
unsigned long prevTime = 0;
float startTime = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  startTime = micros();
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  prevTime = micros();
  
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    //Serial.print(x);
    //Serial.print('\t');
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);

    //Time difference
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    // θg = θ0 + gx * Δt
    gyro_angle = gyro_angle + x * dt;
    Serial.print("Time: ");
    Serial.print((micros() - startTime)/1000000.0);
    Serial.print(" s \t");
    Serial.print(x);
    Serial.print(" deg/s \t Angle: ");
    Serial.print(gyro_angle);
    Serial.println(" deg");

    delay (100);
  }
}
