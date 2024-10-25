#include "ROSutils.hpp"

// output publishers
ros::Publisher pc_pub;
ros::Publisher state_pub;

// debugging publishers
ros::Publisher orig_pub, desk_pub, match_pub, finalraw_pub, body_pub, normals_pub;

// output frames
std::string world_frame, body_frame;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    PointCloudT::Ptr pc_ (boost::make_shared<PointCloudT>());
    pcl::fromROSMsg(*msg, *pc_);

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    loc.updatePointCloud(pc_, msg->header.stamp.toSec());

    // Publish output pointcloud
    sensor_msgs::PointCloud2 pc_ros;
    pcl::toROSMsg(*loc.get_pointcloud(), pc_ros);
    pc_ros.header.stamp = ros::Time::now();
    pc_ros.header.frame_id = world_frame;
    pc_pub.publish(pc_ros);

    // Publish debugging pointclouds
    sensor_msgs::PointCloud2 orig_msg;
    pcl::toROSMsg(*loc.get_orig_pointcloud(), orig_msg);
    orig_msg.header.stamp = ros::Time::now();
    orig_msg.header.frame_id = world_frame;
    orig_pub.publish(orig_msg);

    sensor_msgs::PointCloud2 deskewed_msg;
    pcl::toROSMsg(*loc.get_deskewed_pointcloud(), deskewed_msg);
    deskewed_msg.header.stamp = ros::Time::now();
    deskewed_msg.header.frame_id = world_frame;
    desk_pub.publish(deskewed_msg);

    sensor_msgs::PointCloud2 match_msg;
    pcl::toROSMsg(*loc.get_pc2match_pointcloud(), match_msg);
    match_msg.header.stamp = ros::Time::now();
    match_msg.header.frame_id = body_frame;
    match_pub.publish(match_msg);

    sensor_msgs::PointCloud2 finalraw_msg;
    pcl::toROSMsg(*loc.get_finalraw_pointcloud(), finalraw_msg);
    finalraw_msg.header.stamp = ros::Time::now();
    finalraw_msg.header.frame_id = world_frame;
    finalraw_pub.publish(finalraw_msg);

    sensor_msgs::PointCloud2 normals_msg;
    pcl::toROSMsg(*loc.get_matches_pointcloud(), normals_msg);
    normals_msg.header.stamp = ros::Time::now();
    normals_msg.header.frame_id = world_frame;
    normals_pub.publish(normals_msg);

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();

    fast_limo::IMUmeas imu;
    tf_limo::fromROStoLimo(msg, imu);

    // Propagate IMU measurement
    loc.updateIMU(imu);

    // State publishing
    nav_msgs::Odometry state_msg, body_msg;
    tf_limo::fromLimoToROS(loc.getWorldState(), loc.getPoseCovariance(), loc.getTwistCovariance(), state_msg);
    tf_limo::fromLimoToROS(loc.getBodyState(), loc.getPoseCovariance(), loc.getTwistCovariance(), body_msg);

    // Fill frame id's
    state_msg.header.frame_id = world_frame;
    state_msg.child_frame_id  = body_frame;
    body_msg.header.frame_id  = world_frame;
    body_msg.child_frame_id   = body_frame;

    state_pub.publish(state_msg);
    body_pub.publish(body_msg);

    // TF broadcasting
    tf_limo::broadcastTF(loc.getWorldState(), world_frame, body_frame, true);

}

void mySIGhandler(int sig){
    ros::shutdown();
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters
    fast_limo::Config& config = Config::getInstance();
    config.fill(nh);

    // Read frames names
    nh.param<std::string>("frames/world", world_frame, "map");
    nh.param<std::string>("frames/body", body_frame, "base_link");

    // Define subscribers & publishers
    ros::Subscriber lidar_sub = nh.subscribe(config.topics.lidar, 1, &lidar_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub   = nh.subscribe(config.topics.imu, 1000, &imu_callback, ros::TransportHints().tcpNoDelay());

    pc_pub      = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    state_pub   = nh.advertise<nav_msgs::Odometry>("state", 1);

    // debug
    orig_pub     = nh.advertise<sensor_msgs::PointCloud2>("original", 1);
    desk_pub     = nh.advertise<sensor_msgs::PointCloud2>("deskewed", 1);
    match_pub    = nh.advertise<sensor_msgs::PointCloud2>("match", 1);
    finalraw_pub = nh.advertise<sensor_msgs::PointCloud2>("full_pcl", 1);
    normals_pub  = nh.advertise<sensor_msgs::PointCloud2>("normals", 1);
    body_pub     = nh.advertise<nav_msgs::Odometry>("body_state", 1);

    // Set up fast_limo config
    loc.init();

    // Start spinning (async)
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}