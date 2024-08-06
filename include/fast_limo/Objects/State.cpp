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

#include "fast_limo/Objects/State.hpp"

// class fast_limo::State
    // public

        fast_limo::State::State() : time(0.0) { 
            this->q   = Eigen::Quaternionf::Identity();
            this->p   = Eigen::Vector3f::Zero();
            this->v   = Eigen::Vector3f::Zero();
            this->w   = Eigen::Vector3f::Zero();
            this->a   = Eigen::Vector3f::Zero();
            this->g   = Eigen::Vector3f::Zero();
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

            // Gravity
            this->g = s.grav.get_vect().cast<float>();

            // IMU bias
            this->b.gyro = s.bg.cast<float>();
            this->b.accel = s.ba.cast<float>();

            // Offset LiDAR-IMU
            this->qLI = s.offset_R_L_I.cast<float>();
            this->pLI = s.offset_T_L_I.cast<float>();
        }

        fast_limo::State::State(const state_ikfom& s, double t) : fast_limo::State::State(s) { 
            this->time = t;
        }
        
        fast_limo::State::State(const state_ikfom& s, double t,
                                Eigen::Vector3f a, Eigen::Vector3f w) : fast_limo::State::State(s, t) {
            this->a = a;
            this->w = w;
        }

        fast_limo::State::State(Eigen::Matrix4f& T){

            // Get tranform matrix
            Eigen::Matrix3f R = T.block(0, 0, 3, 3);
            this->q = R;

            this->p = T.block(0, 3, 3, 1);
        }

        void fast_limo::State::update(double t){

                // R ⊞ (w - bw - nw)*dt
                // v ⊞ (R*(a - ba - na) + g)*dt
                // p ⊞ (v*dt + 1/2*(R*(a - ba - na) + g)*dt*dt)

                // Time between IMU samples
                double dt = t - this->time;
                assert(dt > 0);

                // Exp orientation
                Eigen::Vector3f w = this->w - this->b.gyro;
                float w_norm = w.norm();
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

                if (w_norm > 1.e-7){
                    Eigen::Vector3f r = w / w_norm;
                    Eigen::Matrix3f K;

                    K << 0.0, -r[2], r[1],
                         r[2], 0.0, -r[0],
                         -r[1], r[0], 0.0; // skew matrix

                    float r_ang = w_norm * dt;

                    /// Rodrigues Transformation
                    R += std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
                }

                // Acceleration
                Eigen::Vector3f a0 = this->q._transformVector(this->a - this->b.accel);
                a0 += this->g;

                // Orientation
                Eigen::Quaternionf q_update(R);
                this->q *= q_update;

                // Position
                this->p += this->v*dt + 0.5*a0*dt*dt;

                // Velocity
                this->v += a0*dt;

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

            this->g = state.g;
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
