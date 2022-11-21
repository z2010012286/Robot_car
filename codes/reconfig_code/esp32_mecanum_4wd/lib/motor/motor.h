/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 10:56:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-18 16:20:40
 * @FilePath: \esp32_mecanum_4wd\lib\motor\motor.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved. 
 */
#ifndef DEFAULT_MOTOR
#define DEFAULT_MOTOR

#include <Arduino.h>
// #include <Servo.h> 

#include "motor_interface.h"

class Motor: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;

    protected:
        void forward(int pwm) override
        {
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, abs(pwm));
        }

    public:
        Motor( bool invert, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);
            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);

        }
    
        void brake() override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, 0);            
        }
};

#endif