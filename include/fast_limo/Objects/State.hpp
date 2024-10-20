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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "fast_limo/Utils/PCL.hpp"
#include "IKFoM/use-ikfom.hpp"

namespace fast_limo {
class State{

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

		State() : time(0.0) { 
			q   = Eigen::Quaternionf::Identity();
			p   = Eigen::Vector3f::Zero();
			v   = Eigen::Vector3f::Zero();
			w   = Eigen::Vector3f::Zero();
			a   = Eigen::Vector3f::Zero();
			g   = Eigen::Vector3f::Zero();
			pLI = Eigen::Vector3f::Zero();
			qLI = Eigen::Quaternionf::Identity();

			b.gyro  = Eigen::Vector3f::Zero();
			b.accel = Eigen::Vector3f::Zero();
		}


		State(const state_ikfom& s){

			// Odom
			q = s.rot.cast<float>();
			p = s.pos.cast<float>();
			v = s.vel.cast<float>();

			// Gravity
			g = s.grav.get_vect().cast<float>();

			// IMU bias
			b.gyro = s.bg.cast<float>();
			b.accel = s.ba.cast<float>();

			// Offset LiDAR-IMU
			qLI = s.offset_R_L_I.cast<float>();
			pLI = s.offset_T_L_I.cast<float>();
		}

		State(const state_ikfom& s, double t) : State(s) { 
			this->time = t;
		}
		
		State(const state_ikfom& s, double t,
								Eigen::Vector3f a, Eigen::Vector3f w) : State(s, t) {
			this->a = a;
			this->w = w;
		}

		State(Eigen::Matrix4f& T){

			// Get tranform matrix
			Eigen::Matrix3f R = T.block(0, 0, 3, 3);
			this->q = R;

			this->p = T.block(0, 3, 3, 1);
		}

		void update(double t){

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
						 -r[1], r[0], 0.0;

					float r_ang = w_norm * dt;

					/// Rodrigues Transformation
					R += std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
				}

				// Acceleration
				Eigen::Vector3f a0 = this->q._transformVector(this->a - this->b.accel);
				a0 += this->g;

				// Orientation
				Eigen::Quaternionf q_update(R);
				q *= q_update;

				// Position
				p += this->v*dt + 0.5*a0*dt*dt;

				// Velocity
				v += a0*dt;
		}



		Eigen::Matrix4f get_RT(){
			// Transformation matrix
			Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
			T.block(0, 0, 3, 3) = this->q.toRotationMatrix();
			T.block(0, 3, 3, 1) = this->p;

			return T;
		}

		Eigen::Matrix4f get_RT_inv(){
			Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
			Eigen::Matrix3f rot = this->q.toRotationMatrix();
			
			Tinv.block(0, 0, 3, 3) = rot.transpose();
			Tinv.block(0, 3, 3, 1) = -rot.transpose()*this->p;

			return Tinv;
		}

		Eigen::Matrix4f get_extr_RT(){
			// Transform matrix
			Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
			T.block(0, 0, 3, 3) = this->qLI.toRotationMatrix();
			T.block(0, 3, 3, 1) = this->pLI;

			return T;
		}

		Eigen::Matrix4f get_extr_RT_inv(){
			Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
			Eigen::Matrix3f rot = this->qLI.toRotationMatrix();
			
			Tinv.block(0, 0, 3, 3) = rot.transpose();
			Tinv.block(0, 3, 3, 1) = -rot.transpose()*this->pLI;

			return Tinv;
		}

};

	typedef std::vector<State> States;


}

#endif