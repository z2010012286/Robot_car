#include <PIDController.h>
/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pins
*/
#define ENCODER_A 25
#define ENCODER_B 26
/* the MOTOR_CW and MOTOR_CCW pins are used to drive the H-bridge
   the H-bridge then drives the motors, This two pins must have to
   be PWM enabled, otherwise the code will not work.
*/
#define MOTOR_CW 13
#define MOTOR_CCW 12
/*In this section we have defined the gain values for the
   proportional,integral, and derivative controller i have set
   the gain values with the help of trial and error methods.
*/
double Kp = 1; // Proportional constant
double Ki = 1; // Integral Constant
double Kd = 1; // Derivative Constant
#define PWM_OFFSET 50
volatile long int encoder_count = 0; // stores the current encoder count
unsigned int integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 255; // after PID computation data is stored in this variable.
PIDController pidcontroller;
void setup() {
  Serial.begin(115200); // Serial for Debugging
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  /* attach an interrupt to pin ENCODER_A of the Arduino, and when the
      pulse is in the RISING edge called the function encoder().
  */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  pidcontroller.begin(); // initialize the PID instance
  //  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.tune(1, 1, 1); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!
}
void loop() {
  while (Serial.available() > 0) {
    //    integerValue = Serial.parseInt(); // stores the integerValue
    incomingByte = Serial.read(); // stores the /n character
    switch (incomingByte) {
      case '\n':
        continue;
        break;
      case 'q':
        Kp += 0.1;
        break;
      case 'a':
        Kp -= 0.1;
        break;
      case 'w':
        Ki += 0.1;
        break;
      case 's':
        Ki -= 0.1;
        break;
      case 'e':
        Kd += 0.1;
        break;
      case 'd':
        Kd -= 0.1;
        break;
      case 'r':
        integerValue += 1000;
        break;
      case 'f':
        integerValue -= 1000;
        break;

    }
  }
  if (Kp < 0)Kp = 0;
  if (Ki < 0)Ki = 0;
  if (Kd < 0)Kd = 0;
  if (integerValue < 0)integerValue = 0;
  Serial.println("--------------------");
  Serial.print("kp:");
  Serial.print(Kp);
  Serial.print("  ki:");
  Serial.print(Ki);
  Serial.print("  kd:");
  Serial.println(Kd);
  Serial.println("--------------------");
  pidcontroller.tune(Kp, Ki, Kd);
  pidcontroller.setpoint(integerValue); // The "goal" the PID controller tries to "reach",
  Serial.print("收到的设定值：");
  Serial.println(integerValue); // print the incoming value for debugging
  motor_pwm_value = pidcontroller.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output
  Serial.print("pwm计算值：");
  Serial.print(motor_pwm_value); // print the calculated value for debugging
  Serial.print("   ");
  if (motor_pwm_value == 0) {
    analogWrite(MOTOR_CW, 0);
    analogWrite(MOTOR_CCW, 0);
    Serial.println("PWM实际值：0");
  }
  else if (motor_pwm_value > 0) { // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
    motor_ccw(motor_pwm_value);
    Serial.print("PWM实际值：");
    Serial.println(motor_pwm_value);
  }
  else { // else we move it in a counter clockwise direction
    motor_cw(abs(motor_pwm_value));
    Serial.print("PWM实际值：");
    Serial.println(motor_pwm_value);
  }
  Serial.print("编码器值：");
  Serial.println(encoder_count);// print the final encoder count.
  delay(10);
}
void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) // if ENCODER_B is high increase the count
    encoder_count++; // increment the count
  else // else decrease the count
    encoder_count--;  // decrement the count
}
void motor_cw(int power) {
  if (power >= PWM_OFFSET) {
    analogWrite(MOTOR_CW, power); //rotate the motor if the value is grater than 100
    digitalWrite(MOTOR_CCW, LOW); // make the other pin LOW
  }
  else {
    // both of the pins are set to low
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
void motor_ccw(int power) {
  if (power >= PWM_OFFSET) {
    analogWrite(MOTOR_CCW, power);
    digitalWrite(MOTOR_CW, LOW);
  }
  else {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
