/*
 Copyright (c) 2025 Oriol Martínez @fetty31

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
#pragma once

#include <lie_odyssey/lie_odyssey.hpp>

namespace fast_limo::iESEKF {

using Scalar = float;

using Bundle = lie_odyssey::BundleManif<Scalar, 
                                    manif::SGal3,  // pose + velocity 
                                    manif::SE3,    // LiDAR extrinsics
                                    manif::R3,     // angular velocity bias
                                    manif::R3,     // acceleration bias
                                    manif::R3      // gravity (To-Do make it S2 group)
                                    >;

using Group = lie_odyssey::LieGroup<Bundle>;

using Filter = lie_odyssey::iESEKF<Group>;

using Measurement = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using HMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Bundle::DoF>; // Measurement Jacobian (N measurement x Group DoF)

// Propagation model (IMU dynamics)
typename Filter::Tangent f(const Filter& kf, const lie_odyssey::IMUmeas& imu);

// Jacobians of the dynamics
typename Filter::Jacobian df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu);

typename Filter::MappingMatrix df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/);

void H_fun(const Filter& /*kf*/, const Group& X_now, Measurement& z, HMat& H);

} // namespace fast_limo::iESEKF 