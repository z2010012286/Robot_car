#include <Arduino.h>
#include "micro_ros_arduino.h"
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

/// @brief 函数声明
void rclErrorLoop();
void my_errorLoop();
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

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            my_errorLoop();          \
        }                            \
    }                                
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

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




void setup()
{
    unsigned long start = millis();
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
    bool imu_ok = imu.init();
    if (!imu_ok)
    {
        while (1)
        {
            flashLED(3);
        }
    }

    set_microros_transports();
    Serial1.begin(115200, SERIAL_8N1, 33, 32);
    // set_microros_wifi_transports("zhang", "2010012286", "192.168.199.124", 8888);
    state = WAITING_AGENT;
    // unsigned long duration = millis() - start;
    // Serial1.print("setup duration: ");
    // Serial1.println(duration);
}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    // unsigned long start = millis();
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        moveBase();
    }
    // unsigned long duration = millis() - start;
    // Serial1.print("controCallback duration: ");
    // Serial1.println(duration);
}

void odom_publisherCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    unsigned long start = millis();
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        publishOdomData();
    }
    Serial1.print("running time: ");
    Serial1.print(start);
    Serial1.print("     ");
    unsigned long duration = millis() - start;
    Serial1.print("odom_publish duration: ");
    Serial1.println(duration);
}

void imu_publisherCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    // unsigned long start = millis();
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        publishIMUData();
    }
    // unsigned long duration = millis() - start;
    // Serial1.print("imu_publisherCallback duration: ");
    // Serial1.println(duration);
}

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

bool createEntities()
{

    allocator = rcl_get_default_allocator();
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "tianci_robot_node", "", &support));
    // create odometry publisher
    // 似乎odom无法使用best_effort通讯,本硬件配置下~45hz
    // odom设置为20Hz， IMU为50Hz， control为100Hz
    RCCHECK(rclc_publisher_init_best_effort(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 10;        // 100hZ
    const unsigned int odom_publisher_timeout = 50; // 20Hz
    const unsigned int imu_publisher_timeout = 20;  // 50Hz
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    RCCHECK(rclc_timer_init_default(
        &odom_publisher_timer,
        &support,
        RCL_MS_TO_NS(odom_publisher_timeout),
        odom_publisherCallback));
    RCCHECK(rclc_timer_init_default(
        &imu_publisher_timer,
        &support,
        RCL_MS_TO_NS(imu_publisher_timeout),
        imu_publisherCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_publisher_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_publisher_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&odom_publisher_timer);
    rcl_timer_fini(&imu_publisher_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
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

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if (((millis() - prev_cmd_time) >= 300))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);

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

    current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4);
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
    // unsigned long start = millis();
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    // unsigned long odom_rcl_publisher_duration = millis() - start;
    // Serial1.print("odom_publisher RCSOFTCHECK duration: ");
    // Serial1.println(odom_rcl_publisher_duration);
    // Serial1.println(odom_msg.header.stamp.nanosec);
}

void publishIMUData()
{
    imu_msg = imu.getData();
    struct timespec time_stamp = getTime();
    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    // unsigned long start = millis();
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    // unsigned long oimu_rcl_publisher_duration = millis() - start;
    // Serial1.print("imu_publisher RCSOFTCHECK duration: ");
    // Serial1.println(oimu_rcl_publisher_duration);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        Serial1.println("rclError");
    }
}

void my_errorLoop()
{
     
        Serial1.println("soft_error1");
   
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