/*
 Copyright (c) 2024 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

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
        State(Eigen::Matrix4d& s);

        void operator+=(const State& s);

        void update(double t);

        Eigen::Matrix4f get_RT();           // get Rotation & Translation matrix

        Eigen::Matrix4f get_RT_inv();       // get inverted Rotation & Translation matrix

        Eigen::Matrix4f get_extr_RT();      // get estimated extrinsics Rotation & Translation matrix

        Eigen::Matrix4f get_extr_RT_inv();  // get estimated extrinsics inverted Rotation & Translation matrix

};

#endif