#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

#define BUFFER_SIZE 20
// ================== BLE Stuff =========================
// Define a custom BLE service and characteristic
char currentCommand[BUFFER_SIZE + 1] = "BALANCE"; // tracks active state
// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


// =================== PID + IMU Stuff =======================
// ================= MOTOR PINS =================

// Forward declarations
void updateEncoder1();
void updateEncoder2();
void updateTilt();
float MotorController(float error, float dt);
void driveMotors(int pwm);
void MotorTurn(char* turn, int pwm);

const int A_IN1 = 2;
const int A_IN2 = 3;

const int B_IN1 = 5;
const int B_IN2 = 6;

// ================= ENCODER PINS =================

const int encoderA1 = 7;
const int encoderB1 = 8;

const int encoderA2 = 9;
const int encoderB2 = 10;

// ================= ENCODERS =================

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

// ================= RPM =================

long prevCount1 = 0;
long prevCount2 = 0;

float rpmA = 0;
float rpmB = 0;

unsigned long prevRPMTime = 0;

// IMPORTANT: use your measured CPR
const float CPR = 1920.0;

// ================= IMU =================

unsigned long prevTime = 0;
bool firstRun = true; 
float gyroAngle;
float filterAngle;
float gx_d;


// ================= CONTROL =================

float Kp = 12.5; //could try to increase and decrease if this does not work well
float Kd = 0.75; // differential constant 0.6
float Ki = 150.0; // integral constant 
float prevError = 0; // for D
float dt = 0; 
float integral = 0; 
float desiredAngle = 0;
float filteredDerror = 0; // 
float offset; //large pos makes it "balance" tilted to red side, always push battery pack to red bumper for consistency
float alpha = 0.0; // tune this (lower = more smoothing)

// measured deadbands
const int DEAD_A = 46; //with the white bumper closer to you, A is on the right side 
const int DEAD_B = 51; // is the larger one historically (47 by one point)

const int MAX_PWM = 255; 
int pwm; 
//int adjust; 
//int Right_pwm;
//int Left_pwm;  
bool flag1 = true;
bool flag2 = true;
bool flag3 = true;
bool flag4 = true;
bool flag5 = true;
bool flag6 = true;

bool flag_descent = true;

bool flag_ramp_start = true;

long count1 = 0;
long count2 = 0;

//new stuff for ramp:


// =========================================================
// SETUP
// =========================================================

void setup() {
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  // Set the device name and local name
  BLE.setLocalName("BLE-DEVICE");
  BLE.setDeviceName("BLE-DEVICE");
  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);
  // Add the service
  BLE.addService(customService);
  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");
  // Start advertising the service
  BLE.advertise();


  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  pinMode(encoderA1, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);

  // Motor 1 interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA1), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), updateEncoder1, CHANGE);

  // Motor 2 interrupts
  attachInterrupt(digitalPinToInterrupt(encoderA2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), updateEncoder2, CHANGE);

  Wire1.setClock(400000);  // must be before IMU.begin
  Wire.setClock(400000);  // must be before IMU.begin
  IMU.begin();

  if (!IMU.begin()) {
    //Serial.println("IMU failed!");
    while (1);
  }

}

// =========================================================
// LOOP
// =========================================================

void loop() {

  updateTilt();
  offset = 0.50;  

  if (micros()<5000000){encoderCount1 = 0; encoderCount2 = 0;}

  noInterrupts();
  count1 = encoderCount1;
  count2 = encoderCount2;
  interrupts();

  //if (count1 > 100 || count2 > 100) offset = 0.9;  // angle offset positive 
  //if (count1 < -100 || count2 < -100) offset = 0.1; //angle offset negative

  float posError = (count1+count2)/2;
  posError = constrain (posError, -100, 100);
  float posOffset = 0.01*posError;

  /*
  if (micros() > 6000000 && flag_ramp_start) 
    { 
      integral = 0; flag_ramp_start = false; 
      }
  */

  //RAMP YAYAY
  //int target = 5;
  float k;
  //float wheelSpeed;

  int target_5 = 5; 
  int target_10 = 10;
    if (micros()>6000000 && micros()<40000000){
      digitalWrite(LED_BUILTIN, HIGH);
      posOffset = 0;

      k = 0.2; // works at 11V
      if (fabs(desiredAngle - target_5) < 4) k = 0.05;

      /* 
      if (wheelSpeed < 150) {   //tune this threshold
        k = 0.2;
      }
      */

      if (desiredAngle < target_5) {
      desiredAngle += (target_5 - desiredAngle) * k * dt;
      desiredAngle = constrain(desiredAngle, 0, target_5);  // don't overshoot

    }
    }

    else if (micros()>40000000 && micros()<100000000){
      digitalWrite(LED_BUILTIN, HIGH);
      posOffset = 0;

      k = 0.15; // works at 11V
      if (fabs(desiredAngle - target_10) < 8) k = 0.05;

      /* 
      if (wheelSpeed < 150) {   //tune this threshold
        k = 0.2;
      }
      */

      if (desiredAngle < target_10) {
      desiredAngle += (target_10 - desiredAngle) * k * dt;
      desiredAngle = constrain(desiredAngle, 0, target_10);  // don't overshoot

    }
    }

    else if (micros()>100000000){
    digitalWrite(LED_BUILTIN, LOW);

    // One-time reset when descent phase starts
    if (flag_descent) {
        integral = 0;
        encoderCount1 = 0;
        encoderCount2 = 0;
        flag_descent = false;
    }

    float target_down = 0;
    float k_down = 0.15;

    if (fabs(desiredAngle - target_down) > 0.05) {
        desiredAngle += (target_down - desiredAngle) * k_down * dt;
        desiredAngle = constrain(desiredAngle, -1.0, target_10);
    } else {
        desiredAngle = 0;

        encoderCount1 = 0;
        encoderCount2 = 0;
        integral = constrain(integral, -500, 500); // bleed off any integral
    }
    posOffset = 0;  //just trying
    
    // posOffset active, but clamp it so it can't push angle positive
    //posOffset = constrain(posOffset, -0.3, 0.3);
    }

  pwm = (int)MotorController(desiredAngle - filterAngle + offset + posOffset, dt);
        //Serial.println(pwm);
  driveMotors(pwm);


  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    //Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) { 
        int length = customCharacteristic.valueLength(); 
        const unsigned char* receivedData = customCharacteristic.value(); 
        char receivedString[length + 1]; 
        memcpy(receivedString, receivedData, length); 
        receivedString[length] = '\0'; 
        customCharacteristic.writeValue("Data received");
        strncpy(currentCommand, receivedString, BUFFER_SIZE); // ← save it
        //Serial.println(currentCommand);
      }
      // update tilt in every iteration
      updateTilt(); 
      // Then below, always running every loop iteration:
      if (strcmp(currentCommand, "FORWARD") == 0) { 
        // keep fwd
        offset = 0.8; 
        pwm = (int)MotorController(desiredAngle - filterAngle + offset, dt); 
         
        driveMotors(pwm);
        //Serial.println(currentCommand); 
      } 

      else if (strcmp(currentCommand, "REVERSE") == 0) {  
        //keep reversing 
        offset = -0.6;  
        pwm =  (int)MotorController(desiredAngle - filterAngle + offset, dt);
        driveMotors(pwm);
        //Serial.println(currentCommand); 
      }

      else if (strcmp(currentCommand, "RIGHT") == 0) {  
        //keep reversing 
        offset = 0.85;
        pwm =  (int)MotorController(desiredAngle - filterAngle + offset, dt);
        MotorTurn("r", pwm); 
        //Serial.println(currentCommand); 
      }

       else if (strcmp(currentCommand, "LEFT") == 0) {  
        //keep reversing 
        offset = 0.85;
        pwm =  (int)MotorController(desiredAngle - filterAngle + offset, dt);
        MotorTurn("l", pwm); 
        //Serial.println(currentCommand); 
      }
      
      else if (strcmp(currentCommand, "STOP") == 0) { 
        //implement stop 
        //delay (50); 
        driveMotors(0);
        //Serial.println(currentCommand); 
      }

      else if (strcmp(currentCommand, "BALANCE") == 0) {  
        //keep balancing 
        offset = 0.5;  
        pwm = (int)MotorController(desiredAngle - filterAngle + offset, dt);
        Serial.println(pwm);
        driveMotors(pwm);
        //Serial.println(currentCommand); 
      } 
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    //Serial.println("Disconnected from central.");
  }
  
}


void updateTilt() {
  float ax, ay, az;
  float gx, gy, gz;
  float accAngle;

  const float k = 0.99;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){

    //Time difference
    unsigned long currentTime = micros();
    dt = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    // Accelerometer Angle 
    IMU.readAcceleration(ax, ay, az);
    accAngle = atan2 (ay, az) * 180/PI; 
    //Serial.print(accAngle);
    //Serial.print(" deg ");

    //Gyro Angle and Filter Angle
    IMU.readGyroscope(gx, gy, gz);
    gx_d = gx;
    if (firstRun){
      gyroAngle = (accAngle + (gx*(-1)) * dt); // multiplied gx by -1 to align polarity with sensor orientation
      filterAngle = k*gyroAngle + (1-k)*accAngle;
      firstRun = false;
    }
    
    else {
      gyroAngle = (filterAngle + (gx*(-1)) * dt); // multiplied gx by -1 to align polarity with sensor orientation
      filterAngle = k*gyroAngle + (1-k)*accAngle;
    }

  }
  }
// =========================================================
// MOTOR CONTROL WITH INDIVIDUAL DEADBANDS
// =========================================================
void driveMotors(int pwm) {

  int pwmA = pwm;
  int pwmB = pwm;

  if (pwmA > 10) pwmA += DEAD_A;
  else if (pwmA < 10) pwmA -= DEAD_A;

  if (pwmB > 10) pwmB += DEAD_B;
  else if (pwmB < 10) pwmB -= DEAD_B;

  pwmA = constrain(pwmA, -255, 255);
  pwmB = constrain(pwmB, -255, 255);

  float yawError = count1-count2;
  float K_yaw = 0.25;
  pwmA = pwmA-K_yaw*yawError;
  pwmB = pwmB+K_yaw*yawError;

  if (pwm > 0) {

    analogWrite(A_IN1, pwmA);
    analogWrite(A_IN2, 0);

    analogWrite(B_IN1, pwmB);
    analogWrite(B_IN2, 0);

  } else if (pwm < 0) {

    pwmA = -pwmA;
    pwmB = -pwmB;

    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, pwmA);

    analogWrite(B_IN1, 0);
    analogWrite(B_IN2, pwmB);

  } else {

    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, 0);

    analogWrite(B_IN1, 0);
    analogWrite(B_IN2, 0);
  }
}


void updateEncoder1() {

    static int lastA = 0;
    static int lastB = 0;

    int A = digitalRead(encoderA1);
    int B = digitalRead(encoderB1);

    if (A != lastA) {
        if (A == B) encoderCount1++;
        else encoderCount1--;
    }
    else {
        if (A != B) encoderCount1++;
        else encoderCount1--;
    }

    lastA = A;
    lastB = B;
}

void updateEncoder2() {

    static int lastA = 0;
    static int lastB = 0;

    int A = digitalRead(encoderA2);
    int B = digitalRead(encoderB2);

    if (A != lastA) {
        if (A == B) encoderCount2++;
        else encoderCount2--;
    }
    else {
        if (A != B) encoderCount2++;
        else encoderCount2--;
    }

    lastA = A;
    lastB = B;
}


float MotorController(float error, float dt){ // takes tilt angle (error), calculates error based on type of control, outputs pwm 
  // proportional part 
  float P = Kp * error; // or should we do int? 
  //if (error>4) P = Kp* 1.5 * error;
  float D = Kd * gx_d;
  integral = integral + dt*error; 
  //integral = integral + dt*error; 
  integral = constrain(integral, -150, 150);
  if (micros()<5500000) integral = 0;
  float I = integral * Ki;

  if (micros()>10000000 && flag1){integral = 0; flag1 = false;}
  if (micros()>15000000 && flag2){integral = 0; flag2 = false;}
  if (micros()>20000000 && flag3){integral = 0; flag3 = false;}
  if (micros()>25000000 && flag4){integral = 0; flag4 = false;}
  if (micros()>30000000 && flag5){integral = 0; flag5 = false;}
  if (micros()>35000000 && flag6){integral = 0; flag6 = false;}


  float PID = (P + I + D); // change gain later 

  if (micros()<5000000) return 0;
  return PID; 
}

void MotorTurn(char* turn, int pwm){
  // set the right and the left motor to turning values 
  int right_pwm; 
  int left_pwm; 

  right_pwm = constrain(right_pwm, -255, 255);
  left_pwm = constrain(left_pwm, -255, 255);

  if (turn == "r"){
    right_pwm = pwm - 10;
    left_pwm = pwm + 20; 
  }

  else if (turn == "l"){
    right_pwm = pwm +20; 
    left_pwm = pwm -10;
  }

  if (pwm > 0) {
    analogWrite(A_IN1, right_pwm);
    analogWrite(A_IN2, 0);

    analogWrite(B_IN1, left_pwm);
    analogWrite(B_IN2, 0);

  } else if (pwm < 0) {
    right_pwm = -right_pwm;
    left_pwm = -left_pwm;

    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, left_pwm);

    analogWrite(B_IN1, 0);
    analogWrite(B_IN2, right_pwm);

  } else {

    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, 0);

    analogWrite(B_IN1, 0);
    analogWrite(B_IN2, 0);
  }

}
