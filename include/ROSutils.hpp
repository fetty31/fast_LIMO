#pragma once

// Fast LIMO
#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h> 

typedef boost::shared_ptr<std::deque<PointType>> pointDequePtr;
typedef boost::shared_ptr<std::deque<double>> stampDequePtr;

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

}

namespace onethread { 

pcl::PointCloud<PointType>::Ptr getPoints(double t1, double t2, pointDequePtr& buffer){ // HESAI specific

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());

    if(buffer->size() < 1) return pc_;

    // Retreive points from t1 to t2 sorted new to old
    for(int k=0; k < buffer->size(); k++){
        /*To DO: 
            - adapt "timestamp/time" field depending on sensor_type
        */
        PointType p = buffer->at(k);

        if(t1 > p.timestamp) break;
        else if(t2 >= p.timestamp) pc_->points.push_back(p);

    }

    return pc_;
}

pcl::PointCloud<PointType>::Ptr getPoints(double t1, double t2, double& start_time,
                                            pointDequePtr& lidar_buffer, stampDequePtr& stamp_buffer){ // VELODYNE specific

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());

    if(lidar_buffer->size() < 1) return pc_;

    // Retreive points from t1 to t2 sorted new to old
    for(int k=0; k < lidar_buffer->size(); k++){
        PointType p = lidar_buffer->at(k);
        double stamp = stamp_buffer->at(k);
        if(t1 > p.time + stamp) break;
        else if(t2 >= p.time + stamp){
            pc_->points.push_back(p);
            start_time = stamp + p.time;
        }
    }

    return pc_;
}

void clearBuffer(double t, pointDequePtr& buffer){ // HESAI specific
    if(buffer->size() > 0)
        std::cout << std::setprecision(12) << "lidar_buffer.back().timestamp: " << buffer->back().timestamp << std::endl; 

    while(buffer->size() > 0 && t >= buffer->back().timestamp) 
        buffer->pop_back();
}



void clearBuffer(double t, pointDequePtr& lidar_buffer, stampDequePtr& stamp_buffer){ // VELODYNE specific
    if(lidar_buffer->size() > 0)
        std::cout << std::setprecision(12) << "lidar_buffer.back().timestamp: " << lidar_buffer->back().time + stamp_buffer->back() << std::endl; 

    while(lidar_buffer->size() > 0 && t >= lidar_buffer->back().time + stamp_buffer->back()){ 
        lidar_buffer->pop_back();
        stamp_buffer->pop_back();
    }
}

}