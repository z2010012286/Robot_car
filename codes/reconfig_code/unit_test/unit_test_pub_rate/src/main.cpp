#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "base_config.h"
#include "default_imu.h"
#include "ESP32Encoder.h"
#include "odometry.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t odom_publisher_timer;
rcl_timer_t imu_publisher_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Kinematics::velocities current_vel;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

MPU9250IMU imu;
Odometry odometry;

/// @brief 4个编码器实例化
ESP32Encoder motor1_encoder;
ESP32Encoder motor2_encoder;
ESP32Encoder motor3_encoder;
ESP32Encoder motor4_encoder;

/// @brief 4个电机的PWM输出实例化
Motor motor1_controller(MOTOR1_INV, MOTOR1_IN_A, MOTOR1_IN_B, PWMOFFSET);
Motor motor2_controller(MOTOR2_INV, MOTOR2_IN_A, MOTOR2_IN_B, PWMOFFSET);
Motor motor3_controller(MOTOR3_INV, MOTOR3_IN_A, MOTOR3_IN_B, PWMOFFSET);
Motor motor4_controller(MOTOR4_INV, MOTOR4_IN_A, MOTOR4_IN_B, PWMOFFSET);

/// @brief 4个电机的PID控制器实例化
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::MECANUM,
    MOTOR_MAX_RPM *MAX_RPM_RATIO,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE,
    FR_WHEELS_DISTANCE);

/// @brief 函数声明
void rclErrorLoop();
void flashLED(int n_times);
struct timespec getTime();
// void publishData();
bool destroyEntities();
bool createEntities();
void moveBase();
void twistCallback(const void *msgin);
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void publisherCallback(rcl_timer_t *timer, int64_t last_call_time);
void publishOdomData();
void publishIMUData();
void syncTime();
void fullStop();

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

    pinMode(LED_PIN, OUTPUT);
    Wire.begin();
    Serial.begin(115200);
    Serial1.begin(115200,SERIAL_8N1,33,32);
    // Serial2.begin(115200);
    bool imu_ok = imu.init();
    if (!imu_ok)
    {
        while (1)
        {
            flashLED(3);
        }
    }
}

void moveBase(float g_req_linear_vel_x, float g_req_linear_vel_y, float g_req_angular_vel_z)
{
    
    unsigned long now = millis();
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
        unsigned long duration = millis() - now;
    Serial.printf("movebase duration: ");
    Serial.println(duration);
}

void loop()

{
    moveBase(0,0,0);
    publishOdomData();
    publishIMUData();
    Serial1.print("hello");
    delay(10);
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

void publishOdomData()
{
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z);
    odom_msg = odometry.getData();
    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    unsigned long duration = millis() - now;
    Serial.printf("odom duration: ");
    Serial.println(duration);
    //         Serial.printf("odom size: ");
    // Serial.println(sizeof(odom_msg));
}

void publishIMUData()
{
    unsigned long now = millis();
    imu_msg = imu.getData();
    struct timespec time_stamp = getTime();
    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    unsigned long duration = millis() - now;
    Serial.printf("imu duration: ");
    Serial.println(duration);
    //     Serial.printf("imu size: ");
    // Serial.println(sizeof(imu_msg));
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis();
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}