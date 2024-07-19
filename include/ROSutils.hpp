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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h> 

namespace tf_limo {

void fromROStoLimo(const sensor_msgs::Imu::ConstPtr& in, fast_limo::IMUmeas& out){
    out.stamp = in->header.stamp.toSec();

    out.ang_vel(0) = in->angular_velocity.x;
    out.ang_vel(1) = in->angular_velocity.y;
    out.ang_vel(2) = in->angular_velocity.z;

    out.lin_accel(0) = in->linear_acceleration.x;
    out.lin_accel(1) = in->linear_acceleration.y;
    out.lin_accel(2) = in->linear_acceleration.z;

    Eigen::Quaterniond qd;
    tf2::fromMsg(in->orientation, qd);
    out.q = qd.cast<float>();
}

void fromLimoToROS(const fast_limo::State& in, nav_msgs::Odometry& out){
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";

    // Pose/Attitude
    Eigen::Vector3d pos = in.p.cast<double>();
    out.pose.pose.position      = tf2::toMsg(pos);
    out.pose.pose.orientation   = tf2::toMsg(in.q.cast<double>());

    // Twist
    Eigen::Vector3d lin_v = in.v.cast<double>();
    out.twist.twist.linear.x  = lin_v(0);
    out.twist.twist.linear.y  = lin_v(1);
    out.twist.twist.linear.z  = lin_v(2);

    Eigen::Vector3d ang_v = in.w.cast<double>();
    out.twist.twist.angular.x = ang_v(0);
    out.twist.twist.angular.y = ang_v(1);
    out.twist.twist.angular.z = ang_v(2);
}

void fromLimoToROS(const fast_limo::State& in, const std::vector<double>& cov_pose,
                    const std::vector<double>& cov_twist, nav_msgs::Odometry& out){
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";

    // Pose/Attitude
    Eigen::Vector3d pos = in.p.cast<double>();
    out.pose.pose.position      = tf2::toMsg(pos);
    out.pose.pose.orientation   = tf2::toMsg(in.q.cast<double>());

    // Twist
    Eigen::Vector3d lin_v = in.v.cast<double>();
    out.twist.twist.linear.x  = lin_v(0);
    out.twist.twist.linear.y  = lin_v(1);
    out.twist.twist.linear.z  = lin_v(2);

    Eigen::Vector3d ang_v = in.w.cast<double>();
    out.twist.twist.angular.x = ang_v(0);
    out.twist.twist.angular.y = ang_v(1);
    out.twist.twist.angular.z = ang_v(2);

    // Covariances
    for(int i=0; i<cov_pose.size(); i++){
        out.pose.covariance[i]  = cov_pose[i];
        out.twist.covariance[i] = cov_twist[i];
    }
}

void broadcastTF(const fast_limo::State& in, std::string parent_name, std::string child_name, bool now){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp    = (now) ? ros::Time::now() : ros::Time(in.time);
    /* NOTE: depending on IMU sensor rate, the state's stamp could be too old, 
        so a TF warning could be print out (really annoying!).
        In order to avoid this, the "now" argument should be true.
    */
    tf_msg.header.frame_id = parent_name;
    tf_msg.child_frame_id  = child_name;

    // Translation
    Eigen::Vector3d pos = in.p.cast<double>();
    tf_msg.transform.translation.x = pos(0);
    tf_msg.transform.translation.y = pos(1);
    tf_msg.transform.translation.z = pos(2);

    // Rotation
    tf_msg.transform.rotation = tf2::toMsg(in.q.cast<double>());

    // Broadcast
    br.sendTransform(tf_msg);
}

}

namespace visualize_limo {

visualization_msgs::Marker getLocalMapMarker(BoxPointType bb){
    visualization_msgs::Marker m;

    m.ns = "fast_limo";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;

    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 0.5f;

    m.lifetime = ros::Duration(0);
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();

    m.pose.orientation.w = 1.0;

    float x_edge = bb.vertex_max[0] - bb.vertex_min[0];
    float y_edge = bb.vertex_max[1] - bb.vertex_min[1];
    float z_edge = bb.vertex_max[2] - bb.vertex_min[2];

    m.scale.x = x_edge;
    m.scale.y = y_edge;
    m.scale.z = z_edge;

    m.pose.position.x = bb.vertex_min[0] + x_edge/2.0;
    m.pose.position.y = bb.vertex_min[1] + y_edge/2.0;
    m.pose.position.z = bb.vertex_min[2] + z_edge/2.0;

    return m;
}

visualization_msgs::MarkerArray getMatchesMarker(Matches& matches, std::string frame_id){
    visualization_msgs::MarkerArray m_array;
    visualization_msgs::Marker m;

    m_array.markers.reserve(matches.size());

    m.ns = "fast_limo_match";
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    m.lifetime = ros::Duration(0);
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();

    m.pose.orientation.w = 1.0;

    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;

    for(int i=0; i < matches.size(); i++){
        m.id = i;
        Eigen::Vector3f match_p = matches[i].get_point();
        m.pose.position.x = match_p(0);
        m.pose.position.y = match_p(1);
        m.pose.position.z = match_p(2);

        m_array.markers.push_back(m);
    }

    return m_array;
}

}