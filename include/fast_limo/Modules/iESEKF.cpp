#include "fast_limo/Modules/iESEKF.hpp"

using namespace fast_limo::iESEKF;

static typename Filter::Tangent fast_limo::iESEKF::f(const Filter& kf, const IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

	// Build tangent increment xi for SGal(3) group:
	// xi = [ rho(3); nu(3); theta(3); s(1) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	Eigen::Vector3d g = kf.getState().subgroup<4>().coeffs(); 					// gravity vector estimate
	Eigen::Matrix3d R = kf.getState().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
	Eigen::Vector3d v0 = kf.getState().subgroup<0>().linearVelocity(); 		    // velocity estimate

	// rho (position): approximate displacement contribution over this small step
	// t.template segment<3>(0) = v0 + 0.5 *((imu.accel - imu.bias.accel /* -n_a */) + R.transpose() * g) * imu.dt;
	t.template segment<3>(0) = R*v0;

	// nu (linear acceleration contribution)
	t.template segment<3>(3) = (imu.accel - imu.bias.accel /* -n_a */) + R.transpose() * g;

	// theta (angular velocity contribution)
	t.template segment<3>(6) = (imu.gyro - imu.bias.gyro /* -n_w */);

	// s (time)
	t(9) = 1;

    return t; // cast to Tangent
}

static typename Filter::Jacobian fast_limo::iESEKF::df_dx(const Filter& kf, const IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

    Filter::Jacobian Jx = Filter::Jacobian::Zero();

	Eigen::Vector3d g = kf.getState().subgroup<4>().coeffs(); 					// gravity estimate
	Eigen::Matrix3d R = kf.getState().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
	Eigen::Vector3d v = kf.getState().subgroup<0>().linearVelocity(); 		    // velocity estimate

	// position
	Jx.block<3, 3>(0, 3) = R;
	Jx.block<3, 3>(0, 6) = -R * manif::skew(v);

	// velocity 
    Jx.block<3, 3>(3,  6) = -R.transpose()*manif::skew(g);	     // w.r.t R := d(R^t*g)/dR 
    Jx.block<3, 3>(3, 19) = -Eigen::Matrix3d::Identity(); 	     // w.r.t b_a 
    Jx.block<3, 3>(3, 22) =  R.transpose(); 				 	 // w.r.t g

    // rotation
	Jx.block<3, 3>(6, 6)  = -manif::skew(imu.gyro-imu.bias.gyro); // w.r.t R
    Jx.block<3, 3>(6, 16) = -Eigen::Matrix3d::Identity(); 		  // w.r.t b_w

    return Jx;
}
static typename Filter::MappingMatrix fast_limo::iESEKF::df_dw(const Filter& /*kf*/, const IMUmeas& /*imu*/) 
{
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Filter::MappingMatrix Jw = Filter::MappingMatrix::Zero();

    Jw.block<3, 3>(3, 3)  = -Eigen::Matrix3d::Identity(); // w.r.t n_a
    Jw.block<3, 3>(6, 0)  = -Eigen::Matrix3d::Identity(); // w.r.t n_w
    Jw.block<3, 3>(16, 6) =  Eigen::Matrix3d::Identity(); // w.r.t n_{b_w}
    Jw.block<3, 3>(19, 9) =  Eigen::Matrix3d::Identity(); // w.r.t n_{b_a}
    
    return Jw;
}

void fast_limo::iESEKF::H_fun(const Filter& /*kf*/, const Group& X_now, Measurement& z, HMat& H)
{
    fast_limo::Localizer& LOC = fast_limo::Localizer::getInstance();
	fast_limo::Mapper& MAP = fast_limo::Mapper::getInstance();

	// Calculate matches
	Matches matches = MAP.match(
	    fast_limo::State (X_now),
	    LOC.pc2match
	);

	// // Calculate derivatives
	LOC.calculate_H(
	    // Inputs
	    X_now,
	    matches,

	    // Outputs (z := residual vector, H := measurement jacobian dh/dx)
	    z,
	    H
	);
}