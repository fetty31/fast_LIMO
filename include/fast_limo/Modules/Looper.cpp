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

#include "fast_limo/Modules/Looper.hpp"

// class fast_limo::Looper
    // public

        Looper::Looper() {

        }

        void Looper::update(State s, pcl::PointCloud<PointType>::Ptr& pc){

            // To DO: if conditions are met, add keyframe
            
            this->kf_mtx.lock();
            this->keyframes.push_back(std::make_pair(s, pc));
            this->kf_mtx.unlock();
        }

        State Looper::get_state(){
            State s;
            s.p = Eigen::Vector3f(this->out_estimate.translation().x(),
                                  this->out_estimate.translation().y(),
                                  this->out_estimate.translation().z()
                                  );
            s.q = this->out_estimate.rotation().toQuaternion(); // may fail type conversion
            return s;
        }

        void Looper::get_state(State& s){
            s.p = Eigen::Vector3f(this->out_estimate.translation().x(),
                                  this->out_estimate.translation().y(),
                                  this->out_estimate.translation().z()
                                  );
            s.q = this->out_estimate.rotation().toQuaternion(); // may fail type conversion
            return s;
        }


    // private