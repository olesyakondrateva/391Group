/*
 * Deadband Characterization
 * Arduino Nano 33 BLE + 2x DRV8871
 * Motor A: D2 (IN1), D3 (IN2)
 * Motor B: D5 (IN1), D6 (IN2)
 */

const int A_IN1 = 2;
const int A_IN2 = 3;
const int B_IN1 = 5;
const int B_IN2 = 6;

int pwma = 0;
int pwmb = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(A_IN1, OUTPUT); 
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT); 
  pinMode(B_IN2, OUTPUT);


  digitalWrite(A_IN2, LOW); // reverse direction turned off
  digitalWrite(B_IN2, LOW); // reverse direction turned off

  Serial.println("=== Deadband Sweep ===");
  Serial.println("Watch wheels. Send 'a' when Motor A starts, 'b' when Motor B starts.");
  Serial.println();

  bool markedA = false, markedB = false;

  for (int pwm = 0; pwm <= 255; pwm++) {
    if (!markedA) analogWrite(A_IN1, pwm);
    if (!markedB) analogWrite(B_IN1, pwm);

    Serial.print("PWM: "); Serial.println(pwm);
    delay(100);

    // drain all pending characters
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'a' && !markedA) {
        markedA = true;
        analogWrite(A_IN1, 0);
        Serial.print(">>> Motor A deadband = PWM "); Serial.println(pwm);
      }
      if (c == 'b' && !markedB) {
        markedB = true;
        analogWrite(B_IN1, 0);
        Serial.print(">>> Motor B deadband = PWM "); Serial.println(pwm);
      }
    }

    if (markedA && markedB) break;
    delay(80);
  }

  int deadbandA = 44;
  int deadbandB = 36;

  analogWrite(A_IN1, 0);
  analogWrite(B_IN1, 0);
  Serial.println("\nSweep complete.");

  Serial.println("25% duty cycle of usable range");
  pwma = 97;
  pwmb = 91;
  analogWrite(A_IN1, pwma);
  analogWrite(B_IN1, pwmb);
  delay (10000);

  Serial.println("50% duty cycle of usable range");
  pwma = 150;
  pwmb = 146;
  analogWrite(A_IN1, pwma);
  analogWrite(B_IN1, pwmb);
  delay (10000);

  Serial.println("75% duty cycle of usable range");
  pwma = 202;
  pwmb = 200;
  analogWrite(A_IN1, pwma);
  analogWrite(B_IN1, pwmb);
  delay (10000);

  Serial.println("100% duty cycle of usable range");
  pwma = 255;
  pwmb = 255;
  analogWrite(A_IN1, pwma);
  analogWrite(B_IN1, pwmb);
  delay (10000);

}

void loop() {

}