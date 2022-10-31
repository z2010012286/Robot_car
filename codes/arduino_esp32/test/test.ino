#define ENCODER_A 25
#define ENCODER_B 26
#define MOTOR_CW 12
#define MOTOR_CCW 13

volatile long int encoder_count = 0; // stores the current encoder count

void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) // if ENCODER_B is high increase the count
    encoder_count++; // increment the count
  else // else decrease the count
    encoder_count--;  // decrement the count
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Serial for Debugging
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, FALLING);
  //  attachInterrupt(ENCODER_A, encoder, RISING);
  analogWrite(MOTOR_CCW, 120*2);
  digitalWrite(MOTOR_CW, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encoder_count);
  delay(1000);

}
