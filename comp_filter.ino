#include "Arduino_BMI270_BMM150.h"
float gyroAngle = 0.0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  prevTime = micros();
  }

void loop() {
  // put your main code here, to run repeatedly:

  float ax, ay, az;
  float gx, gy, gz;
  float accAngle;
  //Time difference
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // θg = θ0 + gx * Δt
    gyroAngle = (gyroAngle + (gx*(-1)) * dt);
    Serial.print(gyroAngle);
    Serial.print(" ");
    // Serial.println(" deg");
    
  }
    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    accAngle = atan2 (ay, az) * 180/PI; 
    Serial.println(accAngle);
    Serial.print(" ");
    
  }
 delay(100); 
}
