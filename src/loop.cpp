#include "ROSutils.hpp"

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// loop closure publishers
ros::Publisher loop_pub;

// State obj
fast_limo::State st;

// output frames
std::string world_frame, body_frame;

void state_callback(const nav_msgs::Odometry::ConstPtr& msg){

    // Save state
    tf_limo::fromROStoLimo(msg, st);

}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PointCloud<PointType>::Ptr pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(*msg, *pc_);

    // Update iSAM
    fast_limo::Looper& loop = fast_limo::Looper::getInstance();
    loop.update(st, pc_);

}

void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){

    fast_limo::Looper& loop = fast_limo::Looper::getInstance();
    loop.update(msg->latitude, msg->longitude, msg->altitude);

}

void mySIGhandler(int sig){
    ros::shutdown();
}


void load_config(ros::NodeHandle* nh_ptr, fast_limo::LoopConfig* config){

    nh_ptr->param<std::string>("topics/input/GPSfix", config->topics.gnss_fix,  "/gps/fix");

    nh_ptr->param<int>("ScanContext/NUM_EXCLUDE_RECENT",        config->scancontext.NUM_EXCLUDE_RECENT,         50);
    nh_ptr->param<int>("ScanContext/NUM_CANDIDATES_FROM_TREE",  config->scancontext.NUM_CANDIDATES_FROM_TREE,   10);
    nh_ptr->param<int>("ScanContext/PC_NUM_RING",               config->scancontext.PC_NUM_RING,                20);
    nh_ptr->param<int>("ScanContext/PC_NUM_SECTOR",             config->scancontext.PC_NUM_SECTOR,              60);
    nh_ptr->param<float>("ScanContext/PC_MAX_RADIUS",           config->scancontext.PC_MAX_RADIUS,              80.0f);
    nh_ptr->param<float>("ScanContext/SC_THRESHOLD",            config->scancontext.SC_THRESHOLD,               0.2f);
    nh_ptr->param<float>("ScanContext/SEARCH_RATIO",            config->scancontext.SEARCH_RATIO,               0.1f);

    nh_ptr->param<float>("RadiusSearch/RADIUS",                 config->radiussearch.RADIUS,    10.0f);
    nh_ptr->param<bool>("RadiusSearch/active",                  config->radiussearch.active,    true);

    nh_ptr->param<int>("PoseGraph/MinNumStates", config->posegraph.min_num_states, 3);
    nh_ptr->param<std::vector<double>>("PoseGraph/Covariances/Prior", config->posegraph.prior_cov, std::vector<double> (6, 1.e-12) );
    nh_ptr->param<std::vector<double>>("PoseGraph/Covariances/Odom",  config->posegraph.odom_cov,  std::vector<double> (6, 1.e-6) );
    nh_ptr->param<std::vector<double>>("PoseGraph/Covariances/GPS",   config->posegraph.gnss_cov,  {1.e9, 1.e9, 0.01} );
    nh_ptr->param<std::vector<double>>("PoseGraph/Covariances/Loop",  config->posegraph.loop_cov,  std::vector<double> (6, 0.5) );

    nh_ptr->param<float>("ICP/MAX_DIST",        config->icp.MAX_DIST,       150.0f);
    nh_ptr->param<float>("ICP/TF_EPSILON",      config->icp.TF_EPSILON,     1.e-6f);
    nh_ptr->param<float>("ICP/EUC_FIT_EPSILON", config->icp.EUC_FIT_EPSILON,1.e-6);
    nh_ptr->param<float>("ICP/FIT_SCORE",       config->icp.FIT_SCORE,      0.3f);
    nh_ptr->param<int>("ICP/RANSAC_ITERS",      config->icp.RANSAC_ITERS,   0);
    nh_ptr->param<int>("ICP/WINDOW_SIZE",       config->icp.WINDOW_SIZE,    20);
    nh_ptr->param<int>("ICP/MAX_ITERS",         config->icp.MAX_ITERS,      100);

    float diff_norm, diff_yaw;
    nh_ptr->param<float>("KeyFrames/Odom/DIFF_NORM",  diff_norm,  1.5f);
    nh_ptr->param<float>("KeyFrames/Odom/DIFF_YAW",   diff_yaw,   0.5f);
    std::pair<float, float> odom_diff = {diff_norm, diff_yaw};
    config->kf.odom_diff = odom_diff;

    nh_ptr->param<float>("KeyFrames/GPS/DIFF_NORM", config->kf.gnss_diff, 2.5f);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo_looper");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Looper object
    fast_limo::Looper& LOOP = fast_limo::Looper::getInstance();

    // Load config
    fast_limo::LoopConfig config;
    load_config(&nh, &config);

    // Read frames names
    nh.param<std::string>("frames/world", world_frame, "map");
    nh.param<std::string>("frames/body", body_frame, "base_link");

    // Define subscribers & publishers
    ros::Subscriber state_sub = nh.subscribe("/fast_limo/state", 1, &state_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber pc_sub    = nh.subscribe("/fast_limo/pointcloud", 1, &cloud_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber gnss_sub  = nh.subscribe("/kitti/oxts/gps/fix", 1, &gnss_callback, ros::TransportHints().tcpNoDelay());

    loop_pub = nh.advertise<nav_msgs::Odometry>("state", 1);

    // Debug
    ros::Publisher kf_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("kf/pointcloud", 1);
    ros::Publisher sc_pub       = nh.advertise<std_msgs::Float32>("scan_context/result", 1);
    ros::Publisher sc_idx_pub   = nh.advertise<std_msgs::Int32>("scan_context/index", 1);
    ros::Publisher st_marker_pub = nh.advertise<visualization_msgs::Marker>("kf/states", 1);

    ros::Publisher icp_target_pub = nh.advertise<sensor_msgs::PointCloud2>("icp/target", 1);
    ros::Publisher icp_source_pub = nh.advertise<sensor_msgs::PointCloud2>("icp/source", 1);
    ros::Publisher icp_result_pub = nh.advertise<sensor_msgs::PointCloud2>("icp/result", 1);

    LOOP.init(config);

    nav_msgs::Odometry state_msg;
    std_msgs::Float32 sc_msg;
    std_msgs::Int32 sc_idx_msg;

    ros::Rate r(10.0);
    while(ros::ok()){
        ros::spinOnce();

        if(LOOP.solve()){

            tf_limo::fromLimoToROS(LOOP.get_state(), state_msg);
            loop_pub.publish(state_msg);

            // Debug 
                // Accumulated Key Frames
            std::vector<std::pair<State, 
                        pcl::PointCloud<PointType>::Ptr>> kfs = LOOP.getKeyFrames();

            std::vector<State> state_vec;
            state_vec.reserve(kfs.size());

            pcl::PointCloud<PointType> pc;
            for(int i=0; i<kfs.size(); i++){
                pc += *(kfs[i].second);
                state_vec.push_back(kfs[i].first);
            }

                // Pointcloud map from KFs
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(pc, pc_msg);
            pc_msg.header.stamp = ros::Time::now();
            pc_msg.header.frame_id = world_frame;
            kf_cloud_pub.publish(pc_msg);

                // Corrected KF states
            visualization_msgs::Marker st_msg = visualize_limo::getKfMarker(state_vec, world_frame);
            st_marker_pub.publish(st_msg);

                // Scan Context result
            sc_msg.data = LOOP.getScanContextResult();
            sc_pub.publish(sc_msg);

            sc_idx_msg.data = LOOP.getScanContextIndex();
            sc_idx_pub.publish(sc_idx_msg);

                // ICP output
            if(LOOP.hasICPconverged()){
                sensor_msgs::PointCloud2 pc_source_msg;
                pcl::toROSMsg(*LOOP.getICPsource(), pc_source_msg);
                pc_source_msg.header.stamp = ros::Time::now();
                pc_source_msg.header.frame_id = world_frame;
                icp_source_pub.publish(pc_source_msg);

                sensor_msgs::PointCloud2 pc_target_msg;
                pcl::toROSMsg(*LOOP.getICPtarget(), pc_target_msg);
                pc_target_msg.header.stamp = ros::Time::now();
                pc_target_msg.header.frame_id = world_frame;
                icp_target_pub.publish(pc_target_msg);

                sensor_msgs::PointCloud2 pc_result_msg;
                pcl::toROSMsg(*LOOP.getICPresult(), pc_result_msg);
                pc_result_msg.header.stamp = ros::Time::now();
                pc_result_msg.header.frame_id = world_frame;
                icp_result_pub.publish(pc_result_msg);
            }
        }

        r.sleep();
    }

    return 0;
}