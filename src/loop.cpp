#include "ROSutils.hpp"

// loop closure publishers
ros::Publisher loop_pub;

void state_callback(const nav_msgs::Odometry::ConstPtr& msg){

    fast_limo::State st;
    tf_limo::fromROStoLimo(msg, st);

    // Update iSAM
    fast_limo::Looper& loop = fast_limo::Looper::getInstance();
    loop.update(st);

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
    ros::Subscriber gnss_sub  = nh.subscribe("/kitti/oxts/gps/fix", 1, &gnss_callback, ros::TransportHints().tcpNoDelay());

    loop_pub = nh.advertise<nav_msgs::Odometry>("state", 1);

    LOOP.init(/*To DO: config as arg*/);

    ros::Duration(10.0).sleep();

    ros::Rate r(100.0);
    while(ros::ok()){
        ros::spinOnce();

        LOOP.solve();

        nav_msgs::Odometry state_msg;
        tf_limo::fromLimoToROS(LOOP.get_state(), state_msg);
        loop_pub.publish(state_msg);

        r.sleep();
    }

    return 0;
}