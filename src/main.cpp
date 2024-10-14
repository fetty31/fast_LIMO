#include "ROSutils.hpp"

// output publishers
ros::Publisher pc_pub;
ros::Publisher state_pub;

// debugging publishers
ros::Publisher orig_pub, desk_pub, match_pub, finalraw_pub, body_pub, map_bb_pub, match_points_pub;

// output frames
std::string world_frame, body_frame;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
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
    orig_msg.header.frame_id = body_frame;
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

    // Visualize current map size
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
    visualization_msgs::Marker bb_marker = visualize_limo::getLocalMapMarker(map.get_local_map());
    bb_marker.header.frame_id = world_frame;
    map_bb_pub.publish(bb_marker);

    // Visualize current matches
    visualization_msgs::MarkerArray match_markers = visualize_limo::getMatchesMarker(loc.get_matches(), 
                                                                                    world_frame
                                                                                    );
    match_points_pub.publish(match_markers);

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

void load_config(ros::NodeHandle* nh_ptr, fast_limo::Config* config){

    nh_ptr->param<std::string>("topics/input/lidar", config->topics.lidar,  "/velodyne_points");
    nh_ptr->param<std::string>("topics/input/imu",   config->topics.imu,    "/EL/Sensors/vectornav/IMU");

    nh_ptr->param<int>("num_threads", config->num_threads, 10);
    nh_ptr->param<int>("sensor_type", config->sensor_type, 1);

    nh_ptr->param<bool>("debug",    config->debug,      true);
    nh_ptr->param<bool>("verbose",  config->verbose,    true);

    nh_ptr->param<bool>("estimate_extrinsics",  config->ikfom.estimate_extrinsics,  true);
    nh_ptr->param<bool>("time_offset",          config->time_offset,                true);
    nh_ptr->param<bool>("end_of_sweep",         config->end_of_sweep,               false);

    nh_ptr->param<bool>("calibration/gravity_align", config->gravity_align,     true);
    nh_ptr->param<bool>("calibration/accel",         config->calibrate_accel,   true);
    nh_ptr->param<bool>("calibration/gyro",          config->calibrate_gyro,    true);
    nh_ptr->param<double>("calibration/time",        config->imu_calib_time,    3.0);

    nh_ptr->param<std::vector<float>>("extrinsics/imu/t",   config->extrinsics.imu2baselink_t,      {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/imu/R",   config->extrinsics.imu2baselink_R,      std::vector<float> (9, 0.0));
    nh_ptr->param<std::vector<float>>("extrinsics/lidar/t", config->extrinsics.lidar2baselink_t,    {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("extrinsics/lidar/R", config->extrinsics.lidar2baselink_R,    std::vector<float> (9, 0.0));

    nh_ptr->param<std::vector<float>>("intrinsics/accel/bias",  config->intrinsics.accel_bias,  {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/gyro/bias",   config->intrinsics.gyro_bias,   {0.0, 0.0, 0.0});
    nh_ptr->param<std::vector<float>>("intrinsics/accel/sm",    config->intrinsics.imu_sm,      std::vector<float> (9, 0.0));

    nh_ptr->param<bool>("filters/cropBox/active",                config->filters.crop_active,   true);
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/min", config->filters.cropBoxMin,    {-1.0, -1.0, -1.0});
    nh_ptr->param<std::vector<float>>("filters/cropBox/box/max", config->filters.cropBoxMax,    {1.0, 1.0, 1.0});

    nh_ptr->param<bool>("filters/voxelGrid/active",                 config->filters.voxel_active,   true);
    nh_ptr->param<std::vector<float>>("filters/voxelGrid/leafSize", config->filters.leafSize,       {0.25, 0.25, 0.25});

    nh_ptr->param<bool>("filters/minDistance/active",   config->filters.dist_active,    false);
    nh_ptr->param<double>("filters/minDistance/value",  config->filters.min_dist,       4.0);

    nh_ptr->param<bool>("filters/rateSampling/active",  config->filters.rate_active,    false);
    nh_ptr->param<int>("filters/rateSampling/value",    config->filters.rate_value,     4);

    float fov_deg;
    nh_ptr->param<bool>("filters/FoV/active",  config->filters.fov_active,  false);
    nh_ptr->param<float>("filters/FoV/value",  fov_deg,                     360.0f);
    config->filters.fov_angle = fov_deg *M_PI/360.0; // half of FoV (bc. is divided by the x-axis)

    nh_ptr->param<int>("iKFoM/Mapping/NUM_MATCH_POINTS",    config->ikfom.mapping.NUM_MATCH_POINTS, 5);
    nh_ptr->param<int>("iKFoM/MAX_NUM_MATCHES",             config->ikfom.mapping.MAX_NUM_MATCHES,  2000);
    nh_ptr->param<int>("iKFoM/MAX_NUM_PC2MATCH",            config->ikfom.mapping.MAX_NUM_PC2MATCH, 1.e+4);
    nh_ptr->param<double>("iKFoM/Mapping/MAX_DIST_PLANE",   config->ikfom.mapping.MAX_DIST_PLANE,   2.0);
    nh_ptr->param<double>("iKFoM/Mapping/PLANES_THRESHOLD", config->ikfom.mapping.PLANE_THRESHOLD,  5.e-2);
    nh_ptr->param<bool>("iKFoM/Mapping/LocalMapping",       config->ikfom.mapping.local_mapping,    false);

    nh_ptr->param<float>("iKFoM/iKDTree/balance",   config->ikfom.mapping.ikdtree.balance_param,    0.6f);
    nh_ptr->param<float>("iKFoM/iKDTree/delete",    config->ikfom.mapping.ikdtree.delete_param,     0.3f);
    nh_ptr->param<float>("iKFoM/iKDTree/voxel",     config->ikfom.mapping.ikdtree.voxel_size,       0.2f);
    nh_ptr->param<double>("iKFoM/iKDTree/bb_size",  config->ikfom.mapping.ikdtree.cube_size,        300.0);
    nh_ptr->param<double>("iKFoM/iKDTree/bb_range", config->ikfom.mapping.ikdtree.rm_range,         200.0);

    nh_ptr->param<int>("iKFoM/MAX_NUM_ITERS",            config->ikfom.MAX_NUM_ITERS,   3);
    nh_ptr->param<double>("iKFoM/covariance/gyro",       config->ikfom.cov_gyro,        6.e-4);
    nh_ptr->param<double>("iKFoM/covariance/accel",      config->ikfom.cov_acc,         1.e-2);
    nh_ptr->param<double>("iKFoM/covariance/bias_gyro",  config->ikfom.cov_bias_gyro,   1.e-5);
    nh_ptr->param<double>("iKFoM/covariance/bias_accel", config->ikfom.cov_bias_acc,    3.e-4);

    double ikfom_limits;
    nh_ptr->param<double>("iKFoM/LIMITS", ikfom_limits, 1.e-3);
    config->ikfom.LIMITS = std::vector<double> (23, ikfom_limits);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters
    fast_limo::Config config;
    load_config(&nh, &config);

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
    body_pub     = nh.advertise<nav_msgs::Odometry>("body_state", 1);
    map_bb_pub   = nh.advertise<visualization_msgs::Marker>("map/bb", 1);
    match_points_pub = nh.advertise<visualization_msgs::MarkerArray>("match_points", 1);

    // Set up fast_limo config
    loc.init(config);

    // Start spinning (async)
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}