// Fast LIMO
#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

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

ros::Publisher pc_pub;
ros::Publisher state_pub;

// debug
ros::Publisher orig_pub, desk_pub, match_pub, finalraw_pub;

// buffer variables
std::deque<PointType> lidar_buffer;
double last_t2 = -1.0;

void fromROStoLimo(const sensor_msgs::Imu::ConstPtr& in, fast_limo::IMUmeas& out){
    out.stamp = in->header.stamp.toSec();

    out.ang_vel(0) = in->angular_velocity.x;
    out.ang_vel(1) = in->angular_velocity.y;
    out.ang_vel(2) = in->angular_velocity.z;

    out.lin_accel(0) = in->linear_acceleration.x;
    out.lin_accel(1) = in->linear_acceleration.y;
    out.lin_accel(2) = in->linear_acceleration.z;
}

void fromLimoToROS(const fast_limo::State& in, nav_msgs::Odometry& out){
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "limo_world";

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

pcl::PointCloud<PointType>::Ptr getPoints(double t1, double t2){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());

    if(lidar_buffer.size() < 1) return pc_;

    // Retreive points from t1 to t2 sorted new to old
    for(int k=0; k < lidar_buffer.size(); k++){
        /*To DO: 
            - adapt "timestamp" field depending on sensor_type
        */
        PointType p = lidar_buffer[k];
        if(t1 > p.timestamp) break;
        else if(t2 >= p.timestamp) pc_->points.push_back(p);
    }

    return pc_;
}

void clearBuffer(double t){
    while(lidar_buffer.size() > 0 && t >= lidar_buffer.back().timestamp)
        lidar_buffer.pop_back();
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    // Fill buffer
    // #pragma omp parallel for num_threads(omp_get_max_threads())
    for(int i=0; i < pc_->size(); i++)
        lidar_buffer.push_front(pc_->points[i]);

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

    sensor_msgs::PointCloud2 pc_ros;
    pcl::toROSMsg(*loc.get_pointcloud(), pc_ros);
    pc_ros.header.stamp = msg->header.stamp;
    pc_ros.header.frame_id = "limo_world";
    pc_pub.publish(pc_ros);

    sensor_msgs::PointCloud2 orig_msg;
    pcl::toROSMsg(*loc.get_orig_pointcloud(), orig_msg);
    orig_msg.header.stamp = msg->header.stamp;
    orig_msg.header.frame_id = "limo_world";
    orig_pub.publish(orig_msg);

    sensor_msgs::PointCloud2 deskewed_msg;
    pcl::toROSMsg(*loc.get_deskewed_pointcloud(), deskewed_msg);
    deskewed_msg.header.stamp = msg->header.stamp;
    deskewed_msg.header.frame_id = "limo_world";
    desk_pub.publish(deskewed_msg);

    sensor_msgs::PointCloud2 match_msg;
    pcl::toROSMsg(*loc.get_pc2match_pointcloud(), match_msg);
    match_msg.header.stamp = msg->header.stamp;
    match_msg.header.frame_id = "limo_world";
    match_pub.publish(match_msg);

    sensor_msgs::PointCloud2 finalraw_msg;
    pcl::toROSMsg(*loc.get_finalraw_pointcloud(), finalraw_msg);
    finalraw_msg.header.stamp = msg->header.stamp;
    finalraw_msg.header.frame_id = "limo_world";
    finalraw_pub.publish(finalraw_msg);

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

    fast_limo::IMUmeas imu;
    fromROStoLimo(msg, imu);

    loc.updateIMU(imu);

    last_t2 = msg->header.stamp.toSec(); // last time KF updated

    nav_msgs::Odometry state_msg;
    fromLimoToROS(loc.get_state(), state_msg);
    state_pub.publish(state_msg);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh("~");

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters

    // Define subscribers & publishers
    ros::Subscriber lidar_sub = nh.subscribe("/ona2/sensors/pandar_front/cloud", 1000, lidar_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub   = nh.subscribe("/ona2/sensors/imu_front/imu", 1000, imu_callback, ros::TransportHints().tcpNoDelay());

    pc_pub      = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    state_pub   = nh.advertise<nav_msgs::Odometry>("state", 1);

        // debug
    orig_pub     = nh.advertise<sensor_msgs::PointCloud2>("original", 1);
    desk_pub     = nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1);
    match_pub    = nh.advertise<sensor_msgs::PointCloud2>("match", 1);
    finalraw_pub = nh.advertise<sensor_msgs::PointCloud2>("final_raw", 1);


    /*To DO:
        - update Localizer & Mapper parameters (maybe create config struct)
    */ 

    loc.init(0.0); // To DO: is it start time needed??

    double t1, t2;
    double delta = 0.02;

    ros::Rate r(9999);
    while(ros::ok()){

        while(true){
            
            ros::spinOnce();

            /*To DO:
                - accumulate pcl comming from callback
                - (accumulate imu's --> better done inside callback)
                - call localizer::updateIMU() from t1 to t2+
                - call localizer::updatePointCloud() from t1 to t2
            */

            // Define time interval [t1, t2]
            t2 = last_t2;
            t1 = t2 - delta;

            std::cout << "t2: " << t2 << std::endl;
            std::cout << "t1: " << t1 << std::endl;

            if(t1 < 0.0) break;

            // Get points from t1 to t2
            pcl::PointCloud<PointType>::Ptr piepiece_pc = getPoints(t1, t2);

            // Call fast_limo
            if(piepiece_pc->size() > 0)
                loc.updatePointCloud(piepiece_pc, piepiece_pc->points[0].timestamp);

            // Rate sleep
            r.sleep();
        }
    }

    return 0;
}