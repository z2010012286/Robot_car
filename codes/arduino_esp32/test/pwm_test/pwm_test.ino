#define ledR 12
#define ledG 13
#define ENCODER_A 25
#define ENCODER_B 26
int pwm_data=0;
char incomingByte; 
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
  ledcAttachPin(ledR, 1); // assign RGB led pins to channels
  ledcAttachPin(ledG, 2);
  ledcSetup(1, 1000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 1000, 8);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
   while (Serial.available() > 0) {
    //    integerValue = Serial.parseInt(); // stores the integerValue
    incomingByte = Serial.read(); // stores the /n character
    switch (incomingByte) {
      case '\n':
        continue;
        break;
      case 'q':
        pwm_data += 10;
        break;
      case 'a':
        pwm_data -= 10;
        break;
         case 'w':
        pwm_data += 1;
        break;
      case 's':
        pwm_data -= 1;
        break;
    
    }
  }
  if(pwm_data>=0){
  ledcWrite(1, pwm_data);
  ledcWrite(2, 0);}
  else{
    ledcWrite(1, 0);
  ledcWrite(2, abs(pwm_data));}
    
  Serial.println(encoder_count);
  Serial.println(pwm_data);
  delay(100);

}
