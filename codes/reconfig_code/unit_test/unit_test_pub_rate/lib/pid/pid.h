/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-19 20:37:15
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-19 21:43:31
 * @FilePath: \test_mpu9250_with_motor\lib\pid\pid.h
 * @Description: 
 * 
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved. 
 */

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double compute(float setpoint, float measured_value);
        void updateConstants(float kp, float ki, float kd);

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double integral_;
        double derivative_;
        double prev_error_;
};

#endif
