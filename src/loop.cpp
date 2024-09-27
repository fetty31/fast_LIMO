#include "ROSutils.hpp"

// loop closure publishers
ros::Publisher loop_pub;

// State obj
fast_limo::State st;

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

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo_looper");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySIGhandler); // override default ros sigint signal

    // Declare the one and only Looper object
    fast_limo::Looper& LOOP = fast_limo::Looper::getInstance();

    // Define subscribers & publishers
    ros::Subscriber state_sub = nh.subscribe("/fast_limo/state", 1, &state_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber pc_sub = nh.subscribe("/fast_limo/pointcloud", 1, &cloud_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber gnss_sub  = nh.subscribe("/kitti/oxts/gps/fix", 1, &gnss_callback, ros::TransportHints().tcpNoDelay());

    ros::Publisher kf_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    loop_pub = nh.advertise<nav_msgs::Odometry>("state", 1);

    LOOP.init(/*To DO: config as arg*/);

    // ros::Duration(5.0).sleep();

    ros::Rate r(5.0);
    while(ros::ok()){
        ros::spinOnce();

        if(LOOP.solve()){

            nav_msgs::Odometry state_msg;
            tf_limo::fromLimoToROS(LOOP.get_state(), state_msg);
            loop_pub.publish(state_msg);

            std::vector<std::pair<State, 
                        pcl::PointCloud<PointType>::Ptr>> kfs = LOOP.getKeyFrames();

            if(kfs.size() > 0){
                pcl::PointCloud<PointType> pc;
                for(int i=0; i<kfs.size(); i++)
                    pc += *(kfs[i].second);

                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(pc, pc_msg);
                pc_msg.header.stamp = ros::Time::now();
                pc_msg.header.frame_id = "map";
                kf_cloud_pub.publish(pc_msg);
            }
        }

        r.sleep();
    }

    return 0;
}