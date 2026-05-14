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
  const float k =0.9;
  float filterAngle;

  //Time difference
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    // Gyro Angle
    IMU.readGyroscope(gx, gy, gz);
    // θg = θ0 + gx * Δt
    gyroAngle = (gyroAngle + (gx*(-1)) * dt); // multiplied gx by -1 to align polarity with sensor orientation
    Serial.print(gyroAngle);
    Serial.print(" ");
    Serial.println(" deg");

    // Accelerometer Angle 
    IMU.readAcceleration(ax, ay, az);
    accAngle = atan2 (ay, az) * 180/PI; 
    Serial.println(accAngle);
    Serial.print(" ");

    // Complimentary Filter
    filterAngle = k*gyroAngle + (1-k)*accAngle;
    Serial.println(filterAngle);
    Serial.print(" deg");
  }

 delay(100); 
}
