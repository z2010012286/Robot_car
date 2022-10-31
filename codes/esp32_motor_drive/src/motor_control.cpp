/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-10-26 20:44:29
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-10-30 11:20:15
 * @FilePath: \esp32_motor_drive\src\motor_control.cpp
 * @Description:
 *
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved.
 */
#include "constants.h"
void pid_controller_init(PIDController &pid1, PIDController &pid2, PIDController &pid3, PIDController &pid4)
{
    pid1.begin(); // initialize the PID instance
    //  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
    pid1.tune(7.8, 0.05, 61); // Tune the PID, arguments: kP, kI, kD
    pid1.limit(PID_MIN_OUTPUT, PID_MAX_OUTPUT);

    pid2.begin(); // initialize the PID instance
    //  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
    pid2.tune(7.8, 0.05, 61); // Tune the PID, arguments: kP, kI, kD
    pid2.limit(PID_MIN_OUTPUT, PID_MAX_OUTPUT);

    pid3.begin(); // initialize the PID instance
    //  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
    pid3.tune(7.8, 0.05, 61); // Tune the PID, arguments: kP, kI, kD
    pid3.limit(PID_MIN_OUTPUT, PID_MAX_OUTPUT);

    pid4.begin(); // initialize the PID instance
    //  pidcontroller.tune(260, 2.7, 2000); // Tune the PID, arguments: kP, kI, kD
    pid4.tune(7.8, 0.05, 61); // Tune the PID, arguments: kP, kI, kD
    pid4.limit(PID_MIN_OUTPUT, PID_MAX_OUTPUT);
};

void motor_cw(int motor, int power)
{
    if (power >= PWM_OFFSET)
    {
        switch (motor)
        {
        case 1:
            analogWrite(MOTOR1_PWMOUT1, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR1_PWMOUT2, 0);
            break;
        case 2:
            analogWrite(MOTOR2_PWMOUT1, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR2_PWMOUT2, LOW);
            break;
        case 3:
            analogWrite(MOTOR3_PWMOUT1, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR3_PWMOUT2, LOW);
            break;
        case 4:
            analogWrite(MOTOR4_PWMOUT1, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR4_PWMOUT2, LOW);
            break;
        default:
            break;
        }
    }
    else
    {
        switch (motor)
        {
        case 1:
            analogWrite(MOTOR1_PWMOUT1, LOW);
            analogWrite(MOTOR1_PWMOUT2, LOW);
            break;
        case 2:
            analogWrite(MOTOR2_PWMOUT1, LOW);
            analogWrite(MOTOR2_PWMOUT2, LOW);
            break;
        case 3:
            analogWrite(MOTOR3_PWMOUT1, LOW);
            analogWrite(MOTOR3_PWMOUT2, LOW);
            break;
        case 4:
            analogWrite(MOTOR4_PWMOUT1, LOW);
            analogWrite(MOTOR4_PWMOUT2, LOW);
            break;
        default:
            break;
        }
    }
}

void motor_ccw(int motor, int power)
{
    if (power >= PWM_OFFSET)
    {
        switch (motor)
        {
        case 1:
            analogWrite(MOTOR1_PWMOUT2, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR1_PWMOUT1, LOW);
            break;
        case 2:
            analogWrite(MOTOR2_PWMOUT2, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR2_PWMOUT1, LOW);
            break;
        case 3:
            analogWrite(MOTOR3_PWMOUT2, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR3_PWMOUT1, LOW);
            break;
        case 4:
            analogWrite(MOTOR4_PWMOUT2, power); // rotate the motor if the value is grater than 100
            analogWrite(MOTOR4_PWMOUT1, LOW);
            break;
        default:
            break;
        }
    }
    else
    {
        switch (motor)
        {
        case 1:
            analogWrite(MOTOR1_PWMOUT2, LOW);
            analogWrite(MOTOR1_PWMOUT1, LOW);
            break;
        case 2:
            analogWrite(MOTOR2_PWMOUT2, LOW);
            analogWrite(MOTOR2_PWMOUT1, LOW);
            break;
        case 3:
            analogWrite(MOTOR3_PWMOUT2, LOW);
            analogWrite(MOTOR3_PWMOUT1, LOW);
            break;
        case 4:
            analogWrite(MOTOR4_PWMOUT2, LOW);
            analogWrite(MOTOR4_PWMOUT1, LOW);
            break;
        default:
            break;
        }
    }
}

void position_update(PIDController &pid, int motor, volatile long int encoder_count, int target_value)
{
    static int j;
    int motor_pwm_value = 0;
    pid.setpoint(target_value);
    motor_pwm_value = pid.compute(encoder_count);
    if (motor_pwm_value >= 0)
    { // if the motor_pwm_value is greater than zero we rotate the  motor in clockwise direction
        motor_pwm_value = constrain(motor_pwm_value + PWM_OFFSET, 0, PID_MAX_OUTPUT);
        motor_ccw(motor, motor_pwm_value);
        if (j % 30000 == 0)
        {
            Serial.print("PWM实际值:");
            Serial.println(motor_pwm_value);
        }
    }
    else
    { // else we move it in a counter clockwise direction
        motor_pwm_value = constrain(motor_pwm_value - PWM_OFFSET, PID_MIN_OUTPUT, 0);
        motor_cw(motor, abs(motor_pwm_value));
        if (j % 300000 == 0)
        {
            Serial.print("PWM实际值:");
            Serial.println(motor_pwm_value);
        }
    }
    j++;
}