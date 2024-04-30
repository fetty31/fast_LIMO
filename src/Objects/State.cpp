#include "fast_limo/Objects/State.hpp"


fast_limo::State::State(state_ikfom& s){

    // Odom
    this->q = s.rot.cast<float>();
    this->p = s.pos.cast<float>();
    this->v = s.vel.cast<float>();

    // IMU bias
    this->b.gyro = s.bg.cast<float>();
    this->b.accel = s.ba.cast<float>();

    // Offset LiDAR-IMU
    this->qLI = s.offset_R_L_I.cast<float>();
    this->pLI = s.offset_T_L_I.cast<float>();
}

State(Eigen::Matrix4f& T){

    // Get tranform matrix
    Eigen::Matrix3f R = T.block(0, 0, 3, 3);
    this->q = R;

    this->p = T.block(0, 3, 3, 1);
}

Eigen::Matrix4f fast_limo::State::get_RT(){
    // Transformation matrix
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block(0, 0, 3, 3) = this->q.toRotationMatrix();
    T.block(0, 3, 3, 1) = this->p;

    return T;
}

Eigen::Matrix4f fast_limo::State::get_RT_inv(){
    Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rot = this->q.toRotationMatrix();
    
    Tinv.block(0, 0, 3, 3) = rot.transpose();
    Tinv.block(0, 3, 3, 1) = -rot.transpose()*this->p;

    return Tinv
}
