#include "ROSutils.hpp"

namespace ros2wrap {

    class LimoWrapper : public rclcpp::Node
    {
        public:

            LimoWrapper() : Node("fast_limo")
                {
                    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                    "lidar", 1, std::bind(&LimoWrapper::lidar_callback, this, std::placeholders::_1));
                    imu_sub_   = this->create_subscription<sensor_msgs::msg::Imu>(
                                    "imu", 1000, std::bind(&LimoWrapper::imu_callback, this, std::placeholders::_1));
                }

        private:

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

            void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) const {
                return;
            }

            void imu_callback(const sensor_msgs::msg::Imu & msg) const {
                return;
            }

    };

}