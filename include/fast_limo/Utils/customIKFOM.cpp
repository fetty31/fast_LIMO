#include "fast_limo/Utils/customIKFOM.hpp"

#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Localizer.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/Plane.hpp"

void IKFoM::h_share_model(state_ikfom &updated_state, esekfom::dyn_share_datastruct<double> &ekfom_data) 
{
    fast_limo::Localizer& LOC = fast_limo::Localizer::getInstance();
    fast_limo::Mapper& MAP = fast_limo::Mapper::getInstance();

    // Calculate matches
    Matches matches = MAP.match(
    fast_limo::State (updated_state),
    LOC.pc2match
    );

    // // Calculate derivatives
    LOC.calculate_H(
        // Inputs
        updated_state,
        matches,

        // Outputs
        ekfom_data.h_x,
        ekfom_data.h
    );
}

Eigen::Matrix<double, 24, 1> IKFoM::get_f(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    vect3 a_inertial = s.rot * (in.acc-s.ba);
    for(int i = 0; i < 3; i++ ){
        res(i) = s.vel[i];
        res(i + 3) =  omega[i]; 
        res(i + 12) = a_inertial[i] + s.grav[i]; 
    }
    return res;
}

Eigen::Matrix<double, 24, 23> IKFoM::df_dx(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
    cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
    cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
    Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    s.S2_Mx(grav_matrix, vec, 21);
    cov.template block<3, 2>(12, 21) =  grav_matrix; 
    cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
    return cov;
}


Eigen::Matrix<double, 24, 12> IKFoM::df_dw(state_ikfom &s, const input_ikfom &in)
{
    Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
    cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
    cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
    cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
    return cov;
}