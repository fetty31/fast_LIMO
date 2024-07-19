/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// Std utils
#include <signal.h>

// Fast LIMO
#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace ros2wrapper {

    class LimoWrapper : public rclcpp::Node
    {
        public:

        private:

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    };

}