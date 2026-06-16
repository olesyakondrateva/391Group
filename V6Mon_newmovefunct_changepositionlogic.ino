
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
float encoderStartValue;


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
float ENCODER_50CM = 3520;
float leftoverRange;

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
bool firstForwardFlag = true;
bool forwardDone = false;
bool reverseDone = false;
bool stopSequence = false; 


long count1 = 0;
long count2 = 0;

char previousCommand[BUFFER_SIZE + 1] = "";
bool firstCommandLoop = false;

//turning
bool moveStarted = false;
float lastPosition = 0;
float encoderSpeed = 0;


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
/*
  if (micros()<5000000){encoderCount1 =0; encoderCount2 = 0;}
  //wait before balancing
  else if (micros()>5000000 && micros()<13000000){
    offset = 0.50;
    updateTilt();
    nonblockingbalance();
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    updateTilt();
    if (!firstCommandLoop) {
      noInterrupts();
      encoderCount1 = 0;
      encoderCount2 = 0;
      count1 = 0;
      count2 = 0;
      interrupts();
      stopSequence = false;
      moveStarted = false;
      encoderSpeed = 0;
      lastPosition = 0;
      integral = 0;
      firstCommandLoop = true; //using as opposite, just for testing (so i dont forget to change init)
  }
  //driveMotorsForwardBack(0);
  moveToTarget(ENCODER_50CM);
  }
  //balance for small amount of time 
  // go forward and balance 
  //click button reset 
  
*/
///*
  updateTilt();
  offset = 0.50;  
  if (micros()<5000000){encoderCount1 =0; encoderCount2 = 0;}
  noInterrupts();
  count1 = encoderCount1;
  count2 = encoderCount2;
  interrupts();
  float posError = (count1+count2)/2;
  posError = constrain (posError, -100, 100);
  float posOffset = 0.01*posError;
  pwm = (int)MotorController(desiredAngle - filterAngle + offset + posOffset, dt);
        //Serial.println(pwm);
  driveMotors(pwm);
//*/

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
        //compare with previous command
        if (strcmp(currentCommand, previousCommand) != 0) {
          firstCommandLoop = true;
          strncpy(previousCommand, currentCommand, BUFFER_SIZE);
          previousCommand[BUFFER_SIZE] = '\0';
        } else {
          firstCommandLoop = false;
        } //balance+balance make new button to do inbetween reset


      }
      // update tilt in every iteration
      updateTilt(); 
      // Then below, always running every loop iteration:
      if (strcmp(currentCommand, "FORWARD") == 0) { 
        if (firstCommandLoop) {
          noInterrupts();
          encoderCount1 = 0;
          encoderCount2 = 0;
          count1 = 0;
          count2 = 0;
          interrupts();
          stopSequence = false;
        }
        driveMotorsForwardBack(0);

      } 

      else if (strcmp(currentCommand, "REVERSE") == 0) {  
        if (firstCommandLoop) {
          noInterrupts();
          encoderCount1 = 0;
          encoderCount2 = 0;
          count1 = 0;
          count2 = 0;
          interrupts();
          stopSequence = false;
        }
        driveMotorsForwardBack(1);
        
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
        if (firstCommandLoop) {
          noInterrupts();
          encoderCount1 = 0;
          encoderCount2 = 0;
          count1 = 0;
          count2 = 0;
          interrupts();
          stopSequence = false;
        }
        driveMotors(0);
        //Serial.println(currentCommand); 
      }

      else if (strcmp(currentCommand, "BALANCE") == 0) {  
        if (firstCommandLoop) {
          noInterrupts();
          encoderCount1 = 0;
          encoderCount2 = 0;
          count1 = 0;
          count2 = 0;
          interrupts();
          stopSequence = false;
        }
        offset = 0.5;
        nonblockingbalance();
      } 
      firstCommandLoop = false;
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



void moveToTarget(long targetCounts) {
  if (stopSequence ==false) {
    //updateTilt();
    digitalWrite(LED_BUILTIN, LOW);
    noInterrupts();
    count1 = encoderCount1;
    count2 = encoderCount2;
    interrupts();

    // Corrected sign:
    // positive position = forward
    // negative position = backward
    float position = -((count1 + count2) / 2.0); //encoderaverage

    float posError = targetCounts - position; // leftover: large to start, goes to 0

    if (!moveStarted) {
      lastPosition = position;
      encoderSpeed = 0;
      moveStarted = true;
    } else if (dt > 0.001) {
      encoderSpeed = (position-lastPosition)/dt;
    }

   // static float lastPosition = 0;
    //float encoderSpeed = (position - lastPosition) / dt; //how fast we are moving encoders/seconds
    lastPosition = position;

    float Kpos = 0.0007; //P term  
    float Kbrake = 0.000; // D term 0.00012 default, where balance D on way down helps keep upright, this D helps break soon

    float moveOffset = Kpos * posError - Kbrake * encoderSpeed;

    moveOffset = constrain(moveOffset, -1.0, 1.0);

    offset = 0.5 + moveOffset;

    pwm = (int)MotorController(desiredAngle - filterAngle + offset, dt); 

    driveMotors(pwm); //should still correct for yaw, and there should be no position fixing
    //how to start position fixing after certain time? or just go back into balance
    //if (posError > -5 &&posError <5 )
    //if (posError <10) stopSequence = true;

  } else { //stopSequence ==true 
    digitalWrite(LED_BUILTIN, HIGH);
    offset = 0.5;
    nonblockingbalanceTarget(-targetCounts);
  }
}

void driveMotorsForwardBack(int direction) { //place tilt change right before calling this , and take note of startencodervalue(global)
// 0 for forward (more neg, red), 1 for back (more pos, white)
//if red is forward, negative encoder value accum
// trust that both encoders are the same (no yaw), only consider one 
  float encoderAverage = (encoderCount1+encoderCount2)/2;

    if (stopSequence == false) {
      digitalWrite(LED_BUILTIN, LOW);
      if (direction==0) leftoverRange = ENCODER_50CM -10 + encoderAverage; //4000-50+(-done)
      else leftoverRange = ENCODER_50CM -30 - encoderAverage; //4000-50-done

      //updateTilt();

      if (direction == 0) offset = 0.3;//0.3+leftoverRange/2500;//offset = 1;//offset = 0.4 + leftoverRange*leftoverRange/7000000;     //break+speed //0.2+1.6  tilt angle to start, 0.2+0 to end (no opposite correction for breaks)
      else offset = offset = 0.5-(leftoverRange*leftoverRange/50000000);     //0.8-1.6  tilt angle to start, 0.5-0 to end (no opp correction
      //breaking included above 

    // float posError = (count1+count2)/2;
      //posError = constrain (posError, -100, 100);
      //float posOffset = 0.01*posError;
      noInterrupts();
      count1 = encoderCount1;
      count2 = encoderCount2;
      interrupts();

      pwm = (int)MotorController(desiredAngle - filterAngle + offset, dt);
      driveMotors(pwm);

      if (leftoverRange < 0) { 
        stopSequence = true; //definitely first time 
      }
    } //end of driving forward

    else if (stopSequence == true) { 
      digitalWrite(LED_BUILTIN, HIGH);
      if (direction == 0) {
        offset = 0.5;
        nonblockingbalanceTarget(-ENCODER_50CM);
        //forwardDone = true;
      } else {
        offset = 0.5;
        nonblockingbalanceTarget(ENCODER_50CM);
        //reverseDone = true;
      }
    }

} 

void nonblockingbalance() {

  //updateTilt();
  //offset = 0.50;  

  noInterrupts();
  count1 = encoderCount1;
  count2 = encoderCount2;
  interrupts();

  float posError = (count1+count2)/2; //three magic lines
  posError = constrain (posError, -100, 100);
  float posOffset = 0.01*posError; //0.01 beforre, 0.025 when trying with sid 

  pwm = (int)MotorController(desiredAngle - filterAngle + offset + posOffset, dt);
  driveMotors(pwm);
}


void nonblockingbalanceTarget(int encoderGoal) {

  //updateTilt();
  //offset = 0.50;  

  noInterrupts();
  count1 = encoderCount1;
  count2 = encoderCount2;
  interrupts();

  float posError = (count1+count2-2*encoderGoal)/2; //three magic lines
  posError = constrain (posError, -200, 200);
  float posOffset = 0.01*posError;

  pwm = (int)MotorController(desiredAngle - filterAngle + offset + posOffset, dt);
  driveMotors(pwm);
}


void blockingbalance(int durationSeconds) {
  unsigned long startTime = micros();
  unsigned long durationUs = (unsigned long)durationSeconds * 1000000UL;

  encoderCount1 = 0;
  encoderCount2 = 0;

  integral = 0;      // optional but recommended
  prevError = 0;     // optional if you use prevError later

  while (micros() - startTime < durationUs) {
    updateTilt();

    offset = 0.50;

    noInterrupts();
    count1 = encoderCount1;
    count2 = encoderCount2;
    interrupts();

    float posError = (count1 + count2) / 2.0;
    posError = constrain(posError, -100, 100);

    float posOffset = 0.01 * posError;

    pwm = (int)MotorController(desiredAngle - filterAngle + offset + posOffset, dt);

    driveMotors(pwm);
  }

  driveMotors(0); // stop after timed balance
}



