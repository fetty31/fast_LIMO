#pragma once  

#include <Eigen/Dense>

namespace fast_limo {

struct Extrinsics{
    struct SE3 {
        Eigen::Vector3f t;
        Eigen::Matrix3f R;
    };
    SE3 imu2baselink;
    SE3 lidar2baselink;
    Eigen::Matrix4f imu2baselink_T;
    Eigen::Matrix4f lidar2baselink_T;
  };

  struct IMUmeas{
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
    Eigen::Quaternionf q;
  };


}