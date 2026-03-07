#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"

#include "fast_limo/Modules/iESEKF.hpp"

using namespace fast_limo::iESEKF;

typename Filter::Tangent fast_limo::iESEKF::f(const Filter& kf, const lie_odyssey::IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

	// Build tangent increment xi for SGal(3) group:
	// xi = [ rho(3); nu(3); theta(3); s(1) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<4>().coeffs(); 					// gravity vector estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

	// rho (position): zero

	// nu (linear acceleration contribution)
	t.template segment<3>(3) = (imu.accel - imu.bias.accel /* -n_a */).cast<Scalar>() - R.transpose() * g;

	// theta (angular velocity contribution)
	t.template segment<3>(6) = (imu.gyro - imu.bias.gyro /* -n_w */).cast<Scalar>();

	// s (time)
	t(9) = Scalar(1);

    return t; // cast to Tangent
}

typename Filter::Jacobian fast_limo::iESEKF::df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

    Filter::Jacobian Jx = Filter::Jacobian::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<4>().coeffs(); 					// gravity estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

	// velocity 
    Jx.block<3, 3>(3,  6) = -manif::skew(R.transpose() * g);	        // w.r.t R := d(R^t*g)/dR 
    Jx.block<3, 3>(3, 19) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_a 
    Jx.block<3, 3>(3, 22) = -R.transpose(); 				 	 		// w.r.t g

    // rotation
    Jx.block<3, 3>(6, 16) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_w

    return Jx;
}

typename Filter::MappingMatrix fast_limo::iESEKF::df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/) 
{
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Filter::MappingMatrix Jw = Filter::MappingMatrix::Zero();

    Jw.block<3, 3>(3, 3)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_a
    Jw.block<3, 3>(6, 0)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_w
    Jw.block<3, 3>(16, 6) =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_w}
    Jw.block<3, 3>(19, 9) =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_a}
    
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

void fast_limo::iESEKF::degeneracy_callback(const Filter& /*kf*/, Tangent& dx, const MatDoF& HRH)
{
	/* 
		Here we handle degeneracy in SGal3 group (first subgroup of our Bundle state)
	*/
	static constexpr int DoF = 10;

	// Compute pose information matrix
	using Mat = Eigen::Matrix<Scalar,DoF,DoF>;
	Mat H = HRH.template topLeftCorner<DoF,DoF>();

	// Eigen decomposition
	Eigen::SelfAdjointEigenSolver<Mat> es(H);
	const Mat& V = es.eigenvectors();
	const auto& lambda = es.eigenvalues();

	// Adaptive threshold
	Scalar max_lambda = lambda.maxCoeff();
	Scalar threshold = Scalar(1e-6) * max_lambda;

	Scalar cond = lambda.maxCoeff() / lambda.minCoeff();
	if(cond < Scalar(1e6))
		return; // no degeneracy

	// Build projection
	Mat S = Mat::Zero();
	for(int i=0;i<DoF;i++)
	{
		if(lambda(i) >= threshold)
			S(i,i) = 1.0;
	}

	Mat P = V * S * V.transpose();

	auto& dx_vec = dx.coeffs();
	dx_vec.template segment<DoF>(0) =
		P * dx_vec.template segment<DoF>(0); // update tangent vector

}