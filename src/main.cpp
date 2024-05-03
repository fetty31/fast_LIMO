#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"

// void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

// }

// void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

// }

int main(int argc, char** argv) {

    ros::init(argc, argv, "fast_limo");
    ros::NodeHandle nh;

    // Declare the one and only Localizer and Mapper objects
    fast_limo::Localizer& loc = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

    // Setup config parameters

    // Define subscribers

    // Start spinning (async)

    /*To DO:
        - update Localizer & Mapper parameters (maybe create config struct)
        - define callbacks for handling data into Localizer
        - async spinning
    */ 

   std::cout << FAST_LIMO_v << std::endl;

   return 0;
}