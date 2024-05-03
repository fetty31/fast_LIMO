#ifndef __FASTLIMO_STATE_HPP__
#define __FASTLIMO_STATE_HPP__

#include "fast_limo/Common.hpp"

class fast_limo::State{

    public:

        struct IMUbias;

        Eigen::Vector3f p;      // position in global/world frame
        Eigen::Quaternionf q;   // orientation in global/world frame
        Eigen::Vector3f v;      // linear velocity
        
        Eigen::Vector3f w;      // angular velocity (input)

        // Offsets
        Eigen::Quaternionf qLI;
        Eigen::Vector3f pLI;

        struct IMUbias {
            Eigen::Vector3f gyro;
            Eigen::Vector3f accel;
        } b;                    // IMU bias in base_link/body frame 

        State();
        State(const state_ikfom& s);
        State(Eigen::Matrix4f& s);

        Eigen::Matrix4f get_RT(); // get Rotation & Translation matrix

        Eigen::Matrix4f get_RT_inv(); // get inverted Rotation & Translation matrix

};

#endif