/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-10-26 19:49:27
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-10-29 16:54:58
 * @FilePath: \esp32_motor_drive\src\constants.h
 * @Description:
 *
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <PIDController.h>
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

#define ENCODER1_A 25
#define ENCODER1_B 26
#define ENCODER2_A 33
#define ENCODER2_B 32
#define ENCODER3_A 21
#define ENCODER3_B 22
#define ENCODER4_A 23
#define ENCODER4_B 27

#define MOTOR1_PWMOUT1 13
#define MOTOR1_PWMOUT2 12
#define MOTOR2_PWMOUT1 14
#define MOTOR2_PWMOUT2 15
#define MOTOR3_PWMOUT1 16
#define MOTOR3_PWMOUT2 17
#define MOTOR4_PWMOUT1 18
#define MOTOR4_PWMOUT2 19

#define PWM_OFFSET 50
#define PID_MIN_OUTPUT -255
#define PID_MAX_OUTPUT 255

#endif