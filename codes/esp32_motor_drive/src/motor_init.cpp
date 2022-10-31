/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-10-25 21:53:23
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-10-27 21:37:20
 * @FilePath: \esp32_motor_drive\src\motor_init.cpp
 * @Description:
 *
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved.
 */
#include <Arduino.h>
#include "constants.h"
volatile long int encoder1_count;
volatile long int encoder2_count;
volatile long int encoder3_count;
volatile long int encoder4_count;

void encoder_init(int encoder_a, int encoder_b)
{
    pinMode(encoder_a, INPUT); // ENCODER_A as Input
    pinMode(encoder_b, INPUT); // ENCODER_B as Input
}

void encoder1()
{
    if (digitalRead(ENCODER1_B) == HIGH) // if ENCODER_B is high increase the count
        encoder1_count++;                // increment the count
    else                                 // else decrease the count
        encoder1_count--;                // decrement the count
}

void encoder2()
{
    if (digitalRead(ENCODER2_B) == HIGH) // if ENCODER_B is high increase the count
        encoder2_count++;                // increment the count
    else                                 // else decrease the count
        encoder2_count--;                // decrement the count
}

void encoder3()
{
    if (digitalRead(ENCODER3_B) == HIGH) // if ENCODER_B is high increase the count
        encoder3_count++;                // increment the count
    else                                 // else decrease the count
        encoder3_count--;                // decrement the count
}

void encoder4()
{
    if (digitalRead(ENCODER4_B) == HIGH) // if ENCODER_B is high increase the count
        encoder4_count++;                // increment the count
    else                                 // else decrease the count
        encoder4_count--;                // decrement the count
}
void motor_init(int pwm_out1, int pwm_out2, int encoder_a, int encoder_b)
{
    encoder_init(encoder_a, encoder_b);
    pinMode(pwm_out1, OUTPUT); // MOTOR_CW as Output
    pinMode(pwm_out2, OUTPUT); // MOTOR_CW as Output
}

void four_motors_init()
{
    motor_init(MOTOR1_PWMOUT1, MOTOR1_PWMOUT2, ENCODER1_A, ENCODER1_B);
    motor_init(MOTOR2_PWMOUT1, MOTOR2_PWMOUT2, ENCODER2_A, ENCODER2_B);
    motor_init(MOTOR3_PWMOUT1, MOTOR3_PWMOUT2, ENCODER3_A, ENCODER3_B);
    motor_init(MOTOR4_PWMOUT1, MOTOR4_PWMOUT2, ENCODER4_A, ENCODER4_B);

    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER4_A), encoder4, RISING);
}