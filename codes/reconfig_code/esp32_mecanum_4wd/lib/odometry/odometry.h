/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 10:56:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-16 15:31:10
 * @FilePath: \esp32_mecanum_4wd\lib\odometry\odometry.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved. 
 */
// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
// #include <micro_ros_utilities/type_utilities.h>
// #include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>

class Odometry
{
    public:
        Odometry();
        void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
        nav_msgs__msg__Odometry getData();

    private:
        const void euler_to_quat(float x, float y, float z, float* q);

        nav_msgs__msg__Odometry odom_msg_;
        float x_pos_;
        float y_pos_;
        float heading_;
};

#endif