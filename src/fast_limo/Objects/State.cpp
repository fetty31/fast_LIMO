#include "fast_limo/Objects/State.hpp"

// class fast_limo::State
    // public

        fast_limo::State::State(){ 
            this->q   = Eigen::Quaternionf::Identity();
            this->p   = Eigen::Vector3f::Zero();
            this->v   = Eigen::Vector3f::Zero();
            this->w   = Eigen::Vector3f::Zero();
            this->pLI = Eigen::Vector3f::Zero();
            this->qLI = Eigen::Quaternionf::Identity();

            this->b.gyro  = Eigen::Vector3f::Zero();
            this->b.accel = Eigen::Vector3f::Zero();
        }


        fast_limo::State::State(const state_ikfom& s){

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

        fast_limo::State::State(Eigen::Matrix4f& T){

            // Get tranform matrix
            Eigen::Matrix3f R = T.block(0, 0, 3, 3);
            this->q = R;

            this->p = T.block(0, 3, 3, 1);
        }

        void fast_limo::State::operator+=(const fast_limo::State& state){
            this->q *= state.q;
            this->p += state.p;
            this->v += state.v;
            this->w += state.w;

            this->b.gyro = state.b.gyro;
            this->b.accel = state.b.accel;

            this->qLI = state.qLI;
            this->pLI = state.pLI;
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

            return Tinv;
        }

        Eigen::Matrix4f fast_limo::State::get_extr_RT(){
            // Transform matrix
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block(0, 0, 3, 3) = this->qLI.toRotationMatrix();
            T.block(0, 3, 3, 1) = this->pLI;

            return T;
        }

        Eigen::Matrix4f fast_limo::State::get_extr_RT_inv(){
            Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
            Eigen::Matrix3f rot = this->qLI.toRotationMatrix();
            
            Tinv.block(0, 0, 3, 3) = rot.transpose();
            Tinv.block(0, 3, 3, 1) = -rot.transpose()*this->pLI;

            return Tinv;
        }
