#pragma once  

#include <Eigen/Dense>

namespace fast_limo {

  struct IMUmeas{
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
    Eigen::Quaternionf q;
  };


}