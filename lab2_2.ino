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


  analogWrite(A_IN2, 0); // reverse direction turned off
  analogWrite(B_IN2, 0); // reverse direction turned off

  int deadbandA = 44;
  int deadbandB = 36;

  analogWrite(A_IN1, 0);
  analogWrite(B_IN1, 0);

}

void loop() {

  Serial.println("50% duty cycle of usable range");
  pwma = 150;
  pwmb = 146;
  Serial.println("Both Forward");
  analogWrite(A_IN1, pwma);
  analogWrite(A_IN2, LOW);
  analogWrite(B_IN1, pwmb);
  analogWrite(B_IN2, LOW);  
  delay (10000);

  Serial.println("Both Backward");
  analogWrite(A_IN1, LOW);
  analogWrite(A_IN2, pwma);
  analogWrite(B_IN1, LOW);  
  analogWrite(B_IN2, pwmb);  
  delay (10000);

  Serial.println("Left Forward, Right Backward");
  analogWrite(A_IN1, pwma);
  analogWrite(A_IN2, LOW);
  analogWrite(B_IN1, LOW);  
  analogWrite(B_IN2, pwmb);  
  delay (10000);

  Serial.println("Right Forward, Left Backward");
  analogWrite(A_IN1, LOW);
  analogWrite(A_IN2, pwma);
  analogWrite(B_IN1, pwmb);  
  analogWrite(B_IN2, LOW);  
  delay (10000);

}