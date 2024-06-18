#include "ROSutils.hpp"

ros::Publisher pc_pub;
ros::Publisher state_pub;

// debug
ros::Publisher orig_pub, desk_pub, match_pub, finalraw_pub;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    loc.updatePointCloud(pc_, msg->header.stamp.toSec());

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
    tf_limo::fromROStoLimo(msg, imu);

    loc.updateIMU(imu);

    nav_msgs::Odometry state_msg;
    tf_limo::fromLimoToROS(loc.get_state(), state_msg);
    state_msg.header.frame_id = "limo_world";
    state_pub.publish(state_msg);

}

void load_config(ros::NodeHandle* nh_ptr, fast_limo::Config* config){

    nh_ptr->param<std::string>("topics/input/lidar", config->topics.lidar, "/velodyne_points");
    nh_ptr->param<std::string>("topics/input/imu", config->topics.imu, "/EL/Sensors/vectornav/IMU");

    nh_ptr->param<int>("num_threads", config->num_threads, 10);
    nh_ptr->param<int>("sensor_type", config->sensor_type, 1);

    nh_ptr->param<bool>("debug", config->debug, true);
    nh_ptr->param<bool>("verbose", config->verbose, true);

    nh_ptr->param<bool>("estimate_extrinsics", config->ikfom.estimate_extrinsics, true);
    nh_ptr->param<bool>("time_offset", config->time_offset, true);

    nh_ptr->param<bool>("calibration/gravity_align", config->gravity_align, true);
    nh_ptr->param<bool>("calibration/accel", config->calibrate_accel, true);
    nh_ptr->param<bool>("calibration/gyro", config->calibrate_gyro, true);
    nh_ptr->param<double>("calibration/time", config->imu_calib_time, 3.0);

    nh_ptr->param<std::vector<float>>("extrinsics/baselink2imu/t", config->extrinsics.baselink2imu_t, {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/baselink2imu/R", config->extrinsics.baselink2imu_R, std::vector<float> (9, 0.0));
    nh_ptr->param<std::vector<float>>("extrinsics/baselink2lidar/t", config->extrinsics.baselink2lidar_t, {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/baselink2lidar/R", config->extrinsics.baselink2lidar_R, std::vector<float> (9, 0.0));

    nh_ptr->param<std::vector<float>>("intrinsics/accel/bias", config->intrinsics.accel_bias, {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/gyro/bias", config->intrinsics.gyro_bias, {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/accel/sm", config->intrinsics.imu_sm, std::vector<float> (9, 0.0));

    nh_ptr->param<bool>("filters/cropBox/active", config->filters.crop_active, true);
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/min", config->filters.cropBoxMin, {-1.0, -1.0, -1.0});
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/max", config->filters.cropBoxMax, {1.0, 1.0, 1.0});

    nh_ptr->param<bool>("filters/voxelGrid/active", config->filters.voxel_active, true);
    nh_ptr->param<std::vector<float>>("filters/voxelGrid/leafSize", config->filters.leafSize, {0.25, 0.25, 0.25});

    nh_ptr->param<bool>("filters/minDistance/active", config->filters.dist_active, false);
    nh_ptr->param<double>("filters/minDistance/value", config->filters.min_dist, 4.0);

    nh_ptr->param<bool>("filters/rateSampling/active", config->filters.rate_active, false);
    nh_ptr->param<int>("filters/rateSampling/value", config->filters.rate_value, 4);

    nh_ptr->param<int>("iKFoM/MAX_NUM_ITERS", config->ikfom.MAX_NUM_ITERS, 3);
    nh_ptr->param<int>("iKFoM/NUM_MATCH_POINTS", config->ikfom.NUM_MATCH_POINTS, 5);
    nh_ptr->param<double>("iKFoM/MAX_DIST_PLANE", config->ikfom.MAX_DIST_PLANE, 2.0);
    nh_ptr->param<double>("iKFoM/PLANES_THRESHOLD", config->ikfom.PLANE_THRESHOLD, 5.e-2);
    nh_ptr->param<double>("iKFoM/covariance/gyro", config->ikfom.cov_gyro, 6.e-4);
    nh_ptr->param<double>("iKFoM/covariance/accel", config->ikfom.cov_acc, 1.e-2);
    nh_ptr->param<double>("iKFoM/covariance/bias_gyro", config->ikfom.cov_bias_gyro, 1.e-5);
    nh_ptr->param<double>("iKFoM/covariance/bias_accel", config->ikfom.cov_bias_acc, 3.e-4);

    double ikfom_limits;
    nh_ptr->param<double>("iKFoM/LIMITS", ikfom_limits, 1.e-3);
    config->ikfom.LIMITS = std::vector<double> (23, ikfom_limits);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh("~");

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters
    fast_limo::Config config;
    load_config(&nh, &config);

    // Define subscribers & publishers
    ros::Subscriber lidar_sub = nh.subscribe(config.topics.lidar, 1000, lidar_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub   = nh.subscribe(config.topics.imu, 1000, imu_callback, ros::TransportHints().tcpNoDelay());

    pc_pub      = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    state_pub   = nh.advertise<nav_msgs::Odometry>("state", 1);

        // debug
    orig_pub     = nh.advertise<sensor_msgs::PointCloud2>("original", 1);
    desk_pub     = nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1);
    match_pub    = nh.advertise<sensor_msgs::PointCloud2>("match", 1);
    finalraw_pub = nh.advertise<sensor_msgs::PointCloud2>("final_raw", 1);

    loc.init(config, false/*One thread*/);

    // Start spinning (async)
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}