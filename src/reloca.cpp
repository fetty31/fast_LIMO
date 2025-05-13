#include "ROSutils.hpp"

// publishers
ros::Publisher full_map_pub;
ros::ServiceClient pc_client;

// output frames
std::string map_frame, world_frame;

bool not_relocated = true;

void call_service(void){

    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
    reloca.get_full_map_transformed(map);

    fast_limo::SendPointCloud srv;
    pcl::toROSMsg(*map, srv.request.pointcloud);
    srv.request.pointcloud.header.frame_id = map_frame;
    srv.request.pointcloud.header.stamp = ros::Time::now();

    if (pc_client.call(srv)) {
        ROS_INFO("Map sent to the server.");
        if (srv.response.success) {
            ROS_INFO("Relocation completed successfully. :)"); 
            not_relocated = false;
        } else ROS_WARN("Failed to send the map to the server.");
    } else ROS_ERROR("Failed to call service send_pointcloud");

}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();
    reloca.updateCloud(pc_);

    if(reloca.is_relocated()) call_service();
}

void state_callback(const nav_msgs::Odometry::ConstPtr& msg){

    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();
    reloca.updateState(msg);
}

void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();

    std::vector<double> init_state = {msg->pose.pose.position.x, 
                                     msg->pose.pose.position.y, 
                                     msg->pose.pose.position.z, 
                                     0.0, 0.0, 0.0};

    reloca.updateInitialPose(init_state);
}

void tf_broadcast(void){

    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();

    Eigen::Vector3f p = reloca.get_pose();
    Eigen::Quaternionf q = reloca.get_orientation();

    Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rot = q.toRotationMatrix();
                
    Tinv.block(0, 0, 3, 3) = rot;
    Tinv.block(0, 3, 3, 1) = p;

    fast_limo::State state = fast_limo::State(Tinv);

    tf_limo::broadcastTF(state, map_frame, world_frame, true); 

    // Publish the full map
    pcl::PointCloud<PointType>::Ptr full_map(new pcl::PointCloud<PointType>);
    reloca.get_full_map(full_map);
    sensor_msgs::PointCloud2 full_map_msg;
    pcl::toROSMsg(*full_map, full_map_msg);
    full_map_msg.header.frame_id = map_frame;
    full_map_pub.publish(full_map_msg);
    
}

void mySIGhandler(int sig){
    ros::shutdown();
}

void load_config(ros::NodeHandle* nh_ptr, fast_limo::RelocaConfig* config) {

    nh_ptr->param<bool>("mode", config->mode, true);
    nh_ptr->param<std::string>("map_path", config->map_path, "map/map.pcd");
    nh_ptr->param<double>("distance_threshold", config->distance_threshold, 0.3);
    nh_ptr->param<int>("inliers_threshold", config->inliers_threshold, 10);
    nh_ptr->param<double>("score", config->score, 5.0);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo_reloca");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Relocator object
    fast_limo::Relocator& reloca = fast_limo::Relocator::getInstance();

    // Setup config parameters
    fast_limo::RelocaConfig config;
    load_config(&nh, &config);

    // Read frames names
    nh.param<std::string>(ros::this_node::getNamespace() + "/fast_limo/frames/map", map_frame, "reloca_map");
    nh.param<std::string>(ros::this_node::getNamespace() + "/fast_limo/frames/world", world_frame, "odom");

    // Define subscribers & publishers
    ros::Subscriber lidar_sub = nh.subscribe(ros::this_node::getNamespace() + "/fast_limo/final_raw", 1, &lidar_callback);
    ros::Subscriber state_sub = nh.subscribe(ros::this_node::getNamespace() + "/fast_limo/state", 1, &state_callback);
    ros::Subscriber initialpose_sub = nh.subscribe("/initialpose", 1, &initialpose_callback);

    full_map_pub = nh.advertise<sensor_msgs::PointCloud2>("full_map", 1);

    // Define relocation service 
    pc_client = nh.serviceClient<fast_limo::SendPointCloud>(ros::this_node::getNamespace() + "/fast_limo/send_pointcloud");

    // Set up Reloca config
    reloca.init(config);

    ros::Rate rate(10);

    while(ros::ok()) {
        if(not_relocated) ros::spinOnce();
        tf_broadcast();
        rate.sleep();
    }

    return 0;
}
