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

        Looper::Looper() : priorAdded(false),
                           initFlag(false),
                           global_idx(0),
                           out_estimate(gtsam::Pose3( gtsam::Rot3(gtsam::Rot3::Quaternion(1., 0., 0., 0.)), 
                                                      gtsam::Point3(0., 0., 0.) )) { }

        Looper::~Looper(){
            delete iSAM_;
        }

        void Looper::init(){

            // Set up incremental Smoothing And Mapping (iSAM)
            gtsam::ISAM2Params param;
            // param.relinearizeThreshold = 0.01;
            // param.relinearizeSkip = 1;
            this->iSAM_ = new gtsam::ISAM2(param);

            std::cout << "iSAM2 defined\n";

            // Set buffers capacity
            this->keyframes.set_capacity(10000);

            std::cout << "kf buffer defined\n";

            // Define noise models
            gtsam::Vector prior_cov(6);
            prior_cov << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
            prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_cov);

            gtsam::Vector odom_cov(6);
            odom_cov << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
            odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_cov);

            gtsam::Vector gnss_cov(3);
            gnss_cov << 1.e9, 1.e9, 250.0; // GNSS latitude and longitude are not taken into account, we only correct altitude
            // gnss_noise = gtsam::noiseModel::Robust::Create(
            //             gtsam::noiseModel::mEstimator::Cauchy::Create(1), // To DO: try different estimators
            //             gtsam::noiseModel::Diagonal::Variances(gnss_cov) );
            gnss_noise = gtsam::noiseModel::Diagonal::Variances(gnss_cov);

            this->initFlag = true;
        }

        void Looper::solve(){
            if(not this->initFlag) return;
            if(this->init_estimates.size() < 2 /*config.min_number_states*/) return;

            this->graph_mtx.lock(); // avoid adding new state to the graph during iSAM update

            this->iSAM_->update(this->graph, this->init_estimates);
            this->iSAM_->update();

            gtsam::Values isam_estimates = this->iSAM_->calculateEstimate();
            this->out_estimate = isam_estimates.at<gtsam::Pose3>(static_cast<int>(isam_estimates.size())-1);
            
            std::cout << "------------------------ optimzed state -------------------------\n";
            std::cout << isam_estimates.at<gtsam::Pose3>(static_cast<int>(isam_estimates.size())-1) << std::endl;
            std::cout << "-----------------------------------------------------------------\n";

            this->graph.resize(0);
            this->init_estimates.clear();

            this->graph_mtx.unlock();

        }

        void Looper::update(State s, pcl::PointCloud<PointType>::Ptr& pc){

            // To DO: if conditions are met, add keyframe
            
            // this->kf_mtx.lock();
            // this->keyframes.push_front(std::make_pair(s, pc));
            // this->kf_mtx.unlock();

            this->update(s);
        }

        void Looper::update(State s){
            this->update(this->fromLIMOtoGTSAM(s));
        }

        void Looper::update(double lat, double lng, double alt){
            if(not this->gnss_tf.hasLocalAxis()){
                gnss_tf.setInitPose(lat, lng, alt);
                return;
            } 

            if(this->init_estimates.size() < 1) return; // do not include GNSS if there aren't any betweenFactor estimates

            double alt_offset = alt - this->gnss_tf.getInitPose()(2);
            gtsam::Point3 gnss_state(out_estimate.x(), out_estimate.y(), alt_offset);

            std::cout << "altitude offset\n";
            std::cout << alt_offset << std::endl;

            graph_mtx.lock();
            graph.add(gtsam::GPSFactor(global_idx-1, gnss_state, gnss_noise));
            graph_mtx.unlock();
        }

        State Looper::get_state(){
            State s;
            s.p = Eigen::Vector3f(this->out_estimate.translation().x(),
                                  this->out_estimate.translation().y(),
                                  this->out_estimate.translation().z()
                                  );
            s.q = this->out_estimate.rotation().toQuaternion().cast<float>(); 
            return s;
        }

        void Looper::get_state(State& s){
            s.p = Eigen::Vector3f(this->out_estimate.translation().x(),
                                  this->out_estimate.translation().y(),
                                  this->out_estimate.translation().z()
                                  );
            s.q = this->out_estimate.rotation().toQuaternion().cast<float>(); 
        }


    // private

        void Looper::update(gtsam::Pose3 pose){
            graph_mtx.lock();
            if(not this->priorAdded)
            {
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, this->prior_noise));
                init_estimates.insert(0, pose);
                priorAdded = true;
            }
            else
            {
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(global_idx-1, global_idx, out_estimate.between(pose), this->odom_noise));
                init_estimates.insert(global_idx, pose);
            }
            graph_mtx.unlock();

            std::cout << "out estimate\n";
            std::cout << out_estimate << std::endl;

            std::cout << "input estimate\n";
            std::cout << pose << std::endl;

            std::cout << "between pose:\n";
            std::cout << out_estimate.between(pose) << std::endl;

            if(global_idx < UINT64_MAX) global_idx++;
            else{
                std::cout << "-------------------------------------------------------------------\n";
                std::cout << "FAST_LIMO::FATAL ERROR: The graph has reached its biggest dimension! Finishing FASTLIMO process...\n";
                std::cout << "-------------------------------------------------------------------\n";
                exit(0);
            }

        }

        gtsam::Pose3 Looper::fromLIMOtoGTSAM(const State& s){
            return gtsam::Pose3( gtsam::Rot3(gtsam::Rot3::Quaternion(s.q.w(), s.q.x(), s.q.y(), s.q.z())), 
                                 gtsam::Point3(s.p(0), s.p(1), s.p(2)) );
        }