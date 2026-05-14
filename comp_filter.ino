
#include "Arduino_BMI270_BMM150.h"
float gyro_angle = 0.0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  //Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //Serial.print("Gyroscope sample rate = ");
  //Serial.print(IMU.gyroscopeSampleRate());
  //Serial.println(" Hz");
  //Serial.println();
  //Serial.println("Gyroscope in degrees/second");
  //Serial.println("X\tY\tZ");


  //Serial.print("Accelerometer sample rate = ");
  //Serial.print(IMU.accelerationSampleRate());
  //Serial.println(" Hz");
  //Serial.println();
  //Serial.println("Acceleration in G's");
  //Serial.println("X\tY\tZ");

  prevTime = micros();
  }

void loop() {
  // put your main code here, to run repeatedly:

  float x, y, z;
  float tiltAngle;
  float tiltAngledeg;


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
    gyro_angle = (gyro_angle + (x*(-1)) * dt);
    //Serial.print (x);
    //Serial.print(" deg/s \t Angle: ");
    Serial.print(gyro_angle);
    Serial.print(" ");
    // Serial.println(" deg");
    
  }
    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    //Serial.print(x);
    //Serial.print('\t');
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);

    tiltAngle = atan2 (y, z);
    tiltAngledeg = tiltAngle * 180/PI; 
    //Serial.println(tiltAngle);
    Serial.println(tiltAngledeg);
    Serial.print(" ");
    

  }
 delay(100); 
}
