/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-20 10:25:39
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-22 20:58:20
 * @FilePath: \unit_test_encoder\src\main.cpp
 * @Description:
 *
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved.
 */
#include <Arduino.h>
#include "base_config.h"
#include "ESP32Encoder.h"
#include "motor.h"
#include "pid.h"
#include "kinematics.h"
#define LED_PIN 13

ESP32Encoder motor1_encoder;
ESP32Encoder motor2_encoder;
ESP32Encoder motor3_encoder;
ESP32Encoder motor4_encoder;

/// @brief 4个电机的PWM输出实例化
Motor motor1_controller(MOTOR1_INV, MOTOR1_IN_A, MOTOR1_IN_B, PWMOFFSET);
Motor motor2_controller(MOTOR2_INV, MOTOR2_IN_A, MOTOR2_IN_B, PWMOFFSET);
Motor motor3_controller(MOTOR3_INV, MOTOR3_IN_A, MOTOR3_IN_B, PWMOFFSET);
Motor motor4_controller(MOTOR4_INV, MOTOR4_IN_A, MOTOR4_IN_B, PWMOFFSET);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics::velocities current_vel;

Kinematics kinematics(
    Kinematics::MECANUM,
    MOTOR_MAX_RPM*MAX_RPM_RATIO,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE,
    FR_WHEELS_DISTANCE);

void serial_print()
{
  Serial.println("**************");
  Serial.print("motor1_encoder count:  ");
  Serial.println(motor1_encoder.getCount());
  Serial.print("motor2_encoder count:  ");
  Serial.println(motor2_encoder.getCount());
  Serial.print("motor3_encoder count:  ");
  Serial.println(motor3_encoder.getCount());
  Serial.print("motor4_encoder count:  ");
  Serial.println(motor4_encoder.getCount());

  Serial.print("motor1_encoder RPM:  ");
  Serial.println(motor1_encoder.getRPM(COUNTS_PER_REV1));
  Serial.print("motor2_encoder RPM:  ");
  Serial.println(motor2_encoder.getRPM(COUNTS_PER_REV2));
  Serial.print("motor3_encoder RPM:  ");
  Serial.println(motor3_encoder.getRPM(COUNTS_PER_REV3));
  Serial.print("motor4_encoder RPM:  ");
  Serial.println(motor4_encoder.getRPM(COUNTS_PER_REV4));

  Serial.print("linear_x:  ");
  Serial.println(current_vel.linear_x);
  Serial.print("linearyx:  ");
  Serial.println(current_vel.linear_y);
  Serial.print("angular_z:  ");
  Serial.println(current_vel.angular_z);
}

void moveBase(float g_req_linear_vel_x, float g_req_linear_vel_y, float g_req_angular_vel_z)
{
  // get the required rpm for each motor based on required velocities, and base used
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  // get the current speed of each motor
  float current_rpm1 = motor1_encoder.getRPM(COUNTS_PER_REV1);
  float current_rpm2 = motor2_encoder.getRPM(COUNTS_PER_REV2);
  float current_rpm3 = motor3_encoder.getRPM(COUNTS_PER_REV3);
  float current_rpm4 = motor4_encoder.getRPM(COUNTS_PER_REV4);

  // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
  motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
  motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
  motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

  current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
}

void setup()
{
  motor1_encoder.attachFullQuad(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
  motor1_encoder.setCount(0);
  motor2_encoder.attachFullQuad(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);
  motor2_encoder.setCount(0);
  motor3_encoder.attachFullQuad(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B);
  motor3_encoder.setCount(0);
  motor4_encoder.attachFullQuad(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B);
  motor4_encoder.setCount(0);

  motor1_controller.brake();
  motor2_controller.brake();
  motor3_controller.brake();
  motor4_controller.brake();
  Serial.begin(115200);
}

void loop()
{
  static int i;
  // if (i < 200)
  //   moveBase(0, 0.5, 0);
  // if (i > 200 && i < 400)
  //   moveBase(0, -0.5, 0);
  // if (i > 400 && i < 600)
  //   moveBase(0.5, 0, 0);
  // if (i > 600 && i < 800)
  //   moveBase(-0.5, 0, 0);
  // if (i > 800)
  //   moveBase(0, 0, 2);

  delay(10);
  if ((i % 100) == 0)
    serial_print();
  i++;
}