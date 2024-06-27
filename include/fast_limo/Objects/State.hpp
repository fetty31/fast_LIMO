#ifndef __FASTLIMO_STATE_HPP__
#define __FASTLIMO_STATE_HPP__

#include "fast_limo/Common.hpp"

class fast_limo::State{

    public:

        struct IMUbias;

        Eigen::Vector3f p;      // position in global/world frame
        Eigen::Quaternionf q;   // orientation in global/world frame
        Eigen::Vector3f v;      // linear velocity
        Eigen::Vector3f g;      // gravity vector
        
        Eigen::Vector3f w;      // angular velocity (IMU input)
        Eigen::Vector3f a;      // linear acceleration (IMU input)

        // Offsets
        Eigen::Quaternionf qLI;
        Eigen::Vector3f pLI;

        double time;

        struct IMUbias {
            Eigen::Vector3f gyro;
            Eigen::Vector3f accel;
        } b;                    // IMU bias in base_link/body frame 

        State();
        State(const state_ikfom& s);
        State(const state_ikfom& s, double t);
        State(const state_ikfom& s, double t, Eigen::Vector3f a, Eigen::Vector3f w);
        State(Eigen::Matrix4f& s);

        void operator+=(const State& s);

        void update(double t);

        Eigen::Matrix4f get_RT();           // get Rotation & Translation matrix

        Eigen::Matrix4f get_RT_inv();       // get inverted Rotation & Translation matrix

        Eigen::Matrix4f get_extr_RT();      // get estimated extrinsics Rotation & Translation matrix

        Eigen::Matrix4f get_extr_RT_inv();  // get estimated extrinsics inverted Rotation & Translation matrix

};

#endif