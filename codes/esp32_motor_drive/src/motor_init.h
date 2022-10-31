/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-10-25 21:52:39
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-10-27 21:38:02
 * @FilePath: \esp32_motor_drive\src\motor_init.h
 * @Description: 
 * 
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved. 
 */
#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

void encoder_init(int encoder_a, int encoder_b);
void encoder1();
void encoder2();
void encoder3();
void encoder4();
void motor_init(int pwm_out1, int pwm_out2, int encoder_a,int encoder_b);
void four_motors_init();

#endif