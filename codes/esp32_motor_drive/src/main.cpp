#include <Arduino.h>
#include <PIDController.h>
#include "constants.h"
#include "motor_init.h"
#include "motor_control.h"
extern volatile long int encoder1_count;
extern volatile long int encoder2_count;
extern volatile long int encoder3_count;
extern volatile long int encoder4_count; // stores the current encoder count

double Kp = 7.8;  // Proportional constant
double Ki = 0.05; // Integral Constant
double Kd = 61;   // Derivative Constant
hw_timer_t *timer = NULL;

unsigned long int motor1_target = 0;
unsigned long int motor2_target = 0;
unsigned long int motor3_target = 0;
unsigned long int motor4_target = 0;

char incomingByte; // parses and stores each individual character one by one

PIDController pidcontroller1;
PIDController pidcontroller2;
PIDController pidcontroller3;
PIDController pidcontroller4;

void ARDUINO_ISR_ATTR onTimer();
void serial_tune_pid();

void setup()
{
  Serial.begin(115200); // Serial for Debugging
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true); // 1000000为1秒
  timerAlarmEnable(timer);
  four_motors_init();
  pid_controller_init(pidcontroller1, pidcontroller2, pidcontroller3, pidcontroller4);
}

void loop()
{
  serial_tune_pid();
  delay(500);
}

void ARDUINO_ISR_ATTR onTimer()
{
  static int i;
  position_update(pidcontroller1, MOTOR1, encoder1_count, motor1_target);
  position_update(pidcontroller2, MOTOR2, encoder2_count, motor2_target);
  position_update(pidcontroller3, MOTOR3, encoder3_count, motor3_target);
  position_update(pidcontroller4, MOTOR4, encoder4_count, motor4_target);
  if (i % 1 == 0)
  {
    // Serial.print("收到的设定值：");
    // Serial.println(integerValue); // print the incoming value for debugging
    // Serial.print("编码器值：");
    Serial.println(encoder4_count); // print the final encoder count.
  }
  i++;
}

void serial_tune_pid()
{
  while (Serial.available() > 0)
  {
    //    integerValue = Serial.parseInt(); // stores the integerValue
    incomingByte = Serial.read(); // stores the /n character
    switch (incomingByte)
    {
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
      motor1_target += 358;
      motor2_target += 358;
      motor3_target += 358;
      motor4_target += 358; // 358一圈
      break;
    case 'f':
      motor1_target -= 358;
      motor2_target -= 358;
      motor3_target -= 358;
      motor4_target -= 358;
      break;
    case 'p':
      Serial.println("--------------------");
      Serial.print("kp:");
      Serial.print(Kp);
      Serial.print("  ki:");
      Serial.print(Ki);
      Serial.print("  kd:");
      Serial.println(Kd);
      Serial.println("--------------------");
      break;
    }
  }
  if (Kp < 0)
    Kp = 0;
  if (Ki < 0)
    Ki = 0;
  if (Kd < 0)
    Kd = 0;
  if (motor1_target < 0)
    motor1_target = 0;
  if (motor2_target < 0)
    motor2_target = 0;
  if (motor3_target < 0)
    motor3_target = 0;
  if (motor4_target < 0)
    motor4_target = 0;

  pidcontroller1.tune(Kp, Ki, Kd);
}
