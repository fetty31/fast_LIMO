#include "fast_limo/Modules/iESEKF.hpp"

using namespace fast_limo::iESEKF;

// IMU dynamic model

static typename Filter::Tangent fast_limo::iESEKF::f(const Filter& /*f*/, const IMUmeas& imu) 
{
    typename Filter::VecTangent t = Filter::VecTangent::Zero();
    // To-Do
    return t; // cast
}

// Jacobians of the dynamics
static typename Filter::Jacobian fast_limo::iESEKF::df_dx(const Filter&, const IMUmeas&) 
{
    // To-Do
    return Filter::Jacobian::Identity();
}
static typename Filter::MappingMatrix fast_limo::iESEKF::df_dw(const Filter&, const IMUmeas&) 
{
    // To-Do
    return Filter::MappingMatrix::Zero();
}

// Measurement (update) functions

void fast_limo::iESEKF::H_fun(const Filter& /*f*/, const Bundle& X_now, Measurement& z, HMat& H)
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

	    // Outputs
	    H,
	    z
	);
}