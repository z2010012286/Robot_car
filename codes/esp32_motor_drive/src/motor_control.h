/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-10-26 20:44:20
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-10-27 21:36:16
 * @FilePath: \esp32_motor_drive\src\motor_control.h
 * @Description: 
 * 
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved. 
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H


void pid_controller_init(PIDController &pid1, PIDController &pid2,PIDController &pid3,PIDController &pid4);
void motor_cw(int motor, int power);
void motor_ccw(int motor, int power);
void position_update(PIDController &pid, int motor, volatile long int encoder_count, int target_value);

#endif