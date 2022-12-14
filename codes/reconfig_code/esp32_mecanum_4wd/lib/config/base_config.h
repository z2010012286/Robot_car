/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-16 16:27:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-18 15:24:01
 * @FilePath: \esp32_mecanum_4wd\lib\config\base_config.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved. 
 */

#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//define your robot' specs here
#define MOTOR_MAX_RPM 140                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 378              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 378              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 378             // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 378             // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.152                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.271            // distance between left and right wheels

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false 
#define MOTOR2_ENCODER_INV false 
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

//esp32 dev??????I2C SDA SCL???21 22????????????????????????????????????????????????????????????????????????????????????
// ENCODER PINS
#define MOTOR1_ENCODER_A 23
#define MOTOR1_ENCODER_B 24

#define MOTOR2_ENCODER_A 34
#define MOTOR2_ENCODER_B 35

#define MOTOR3_ENCODER_A 4
#define MOTOR3_ENCODER_B 5 

#define MOTOR4_ENCODER_A 26
#define MOTOR4_ENCODER_B 25

// MOTOR PINS
#define MOTOR1_IN_A 16
#define MOTOR1_IN_B 17 

#define MOTOR2_IN_A 14
#define MOTOR2_IN_B 15

#define MOTOR3_IN_A 18
#define MOTOR3_IN_B 19

#define MOTOR4_IN_A 12
#define MOTOR4_IN_B 27

#define PWM_MAX 255
#define PWM_MIN -PWM_MAX


#endif
