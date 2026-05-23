/*
 * =========================================================
 * LAB 2 - PART 5
 * Proportional Motor Control Using Tilt Angle
 * =========================================================
 */

#include "Arduino_BMI270_BMM150.h"

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

unsigned long prevIMUTime = 0;

float gyroAngle = 0.0;
float filterAngle = 0.0;
float tiltAngle = 0.0;

// ================= CONTROL =================

float Kp = 8.0; //could try to increase and decrease if this does not work well

// measured deadbands
const int DEAD_A = 36;
const int DEAD_B = 44;

const int MAX_PWM = 255;

// =========================================================
// SETUP
// =========================================================

void setup() {

  Serial.begin(9600);
  while (!Serial);

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

  if (!IMU.begin()) {
    Serial.println("IMU failed!");
    while (1);
  }

  prevIMUTime = micros();

  Serial.println("System Ready");
}

// =========================================================
// LOOP
// =========================================================

void loop() {

  updateTilt();

  // -------- proportional control --------
  int pwm = (int)(Kp * tiltAngle);
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  driveMotors(pwm);

  // -------- RPM every 100 milli seconds = 0.1 seconds --------
  unsigned long now = millis();

  if (now - prevRPMTime >= 100) {

    noInterrupts();
    long c1 = encoderCount1;
    long c2 = encoderCount2;
    interrupts();

    long d1 = c1 - prevCount1;
    long d2 = c2 - prevCount2;

    rpmA = (d1 * 600.0) / CPR;
    rpmB = (d2 * 600.0) / CPR;

    prevCount1 = c1;
    prevCount2 = c2;
    prevRPMTime = now;

    Serial.print("Angle: ");
    Serial.print(tiltAngle);
    Serial.print(" | PWM: ");
    Serial.print(pwm);
    Serial.print(" | RPM A: ");
    Serial.print(rpmA);
    Serial.print(" | RPM B: ");
    Serial.println(rpmB);
    delay (1000);
  }
}

// =========================================================
// IMU + COMPLEMENTARY FILTER (from Lab 1)
// =========================================================

void updateTilt() {

  float ax, ay, az;
  float gx, gy, gz;

  const float k = 0.9;  //can tune to 0.91, 0.95, 0.98, 0.99, etc. later

  unsigned long now = micros();
  float dt = (now - prevIMUTime) / 1000000.0;
  prevIMUTime = now;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

    IMU.readAcceleration(ax, ay, az);
    float accAngle = atan2(ay, az) * 180 / PI;

    IMU.readGyroscope(gx, gy, gz);

    gyroAngle = filterAngle + (gx * -1) * dt; //"-1" depends on sensor orientation

    filterAngle = k * gyroAngle + (1 - k) * accAngle;

    tiltAngle = filterAngle;
  }
}

// =========================================================
// MOTOR CONTROL WITH INDIVIDUAL DEADBANDS
// =========================================================

void driveMotors(int pwm) {

  int pwmA = pwm;
  int pwmB = pwm;

  if (pwmA > 0) pwmA += DEAD_A;
  else if (pwmA < 0) pwmA -= DEAD_A;

  if (pwmB > 0) pwmB += DEAD_B;
  else if (pwmB < 0) pwmB -= DEAD_B;

  pwmA = constrain(pwmA, -255, 255);
  pwmB = constrain(pwmB, -255, 255);

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

// =========================================================
// ENCODERS (X2 SIMPLE + RELIABLE)
// =========================================================

/* x2 decoding
void updateEncoder1() {

  int A = digitalRead(encoderA1);
  int B = digitalRead(encoderB1);

  if (A == B) encoderCount1++;
  else encoderCount1--;
}

void updateEncoder2() {

  int A = digitalRead(encoderA2);
  int B = digitalRead(encoderB2);

  if (A == B) encoderCount2++;
  else encoderCount2--;
}*/

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
