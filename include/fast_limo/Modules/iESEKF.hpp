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

#include <lie_odyssey.hpp>

namespace fast_limo::iESEKF {

using Bundle = lie_odyssey::BundleManif<float, 
                                    manif::SGal3,  // pose + velocity 
                                    manif::SE3,    // LiDAR extrinsics
                                    manif::R3,     // angular velocity bias
                                    manif::R3,     // acceleration bias
                                    manif::R3      // gravity (To-Do make it S2 group)
                                    >;

using Filter = lie_odyssey::iESEKF<Bundle>;
using Scalar = typename Filter::Scalar;
static constexpr int DoF = Filter::DoF;

using Measurement = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using HMat = Eigen::Matrix<Scalar, Eigen::Dynamic, DoF>; // Measurement Jacobian (N measurement x Group DoF)

// Propagation model (IMU dynamics)
static typename Filter::Tangent f(const Filter& /*f*/, const IMUmeas& imu);

// Jacobians of the dynamics
static typename Filter::Jacobian df_dx(const Filter&, const IMUmeas&);

static typename Filter::MappingMatrix df_dw(const Filter&, const IMUmeas&);

void H_fun(const Filter& /*f*/, const Bundle& /*X_now*/, Measurement& /*z*/, HMat& /*H*/);

} // namespace fast_limo::iESEKF 