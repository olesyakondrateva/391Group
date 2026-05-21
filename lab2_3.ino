/*
 * Encoder Manual Test
 * Arduino Nano 33 BLE + 2x DRV8871 (motors OFF for this test)
 * Motor A: D2 (IN1), D3 (IN2)
 * Motor B: D5 (IN1), D6 (IN2)
 *
 * Encoder inputs (CHANGE THESE if needed):
 * Motor A encoder: D7 (A), D8 (B)
 * Motor B encoder: D9 (A), D10 (B)
 */

// Motor driver pins (not used in this test, but kept for reference)
const int A_IN1 = 2;
const int A_IN2 = 3;
const int B_IN1 = 5;
const int B_IN2 = 6;

// Encoder pins (EDIT if your wiring differs)
const int A_ENC_A = 7;
const int A_ENC_B = 8;
const int B_ENC_A = 9;
const int B_ENC_B = 10;

// Encoder counts
volatile long countA = 0;
volatile long countB = 0;

// Interrupt for Motor A encoder
void ISR_A() {
  if (digitalRead(A_ENC_A) == digitalRead(A_ENC_B)) {
    countA++;
  } else {
    countA--;
  }
}

// Interrupt for Motor B encoder
void ISR_B() {
  if (digitalRead(B_ENC_A) == digitalRead(B_ENC_B)) {
    countB++;
  } else {
    countB--;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Motor pins (kept OFF)
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN1, LOW);
  digitalWrite(B_IN2, LOW);

  // Encoder pins
  pinMode(A_ENC_A, INPUT_PULLUP);
  pinMode(A_ENC_B, INPUT_PULLUP);
  pinMode(B_ENC_A, INPUT_PULLUP);
  pinMode(B_ENC_B, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(A_ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_ENC_A), ISR_B, CHANGE);

  Serial.println("Encoder Manual Test Started");
  Serial.println("Rotate wheels manually...");
}

void loop() {

  static long lastA = 0;
  static long lastB = 0;

  // Print only if values change (reduces spam)
  if (countA != lastA || countB != lastB) {
    Serial.print("Motor A Count: ");
    Serial.print(countA);
    Serial.print(" | Motor B Count: ");
    Serial.println(countB);

    lastA = countA;
    lastB = countB;
  }

  delay(10);
}