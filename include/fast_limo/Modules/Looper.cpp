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
                           last_enu(Eigen::Vector3d::Zero()),
                           out_estimate(gtsam::Pose3( gtsam::Rot3(gtsam::Rot3::Quaternion(1., 0., 0., 0.)), 
                                                      gtsam::Point3(0., 0., 0.) )) { }

        Looper::~Looper(){
            delete iSAM_;
        }

        void Looper::init(){

            // Set up incremental Smoothing And Mapping (iSAM)
            gtsam::ISAM2Params param;
            param.relinearizeThreshold = 0.01;
            param.relinearizeSkip = 1;
            this->iSAM_ = new gtsam::ISAM2(param);

            // Set buffers capacity
            this->keyframes.set_capacity(10000);

            // Define noise models
            gtsam::Vector prior_cov(6);
            prior_cov << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
            prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_cov);

            gtsam::Vector odom_cov(6);
            odom_cov << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
            odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_cov);

            gtsam::Vector gnss_cov(3);
            gnss_cov << 1.e9, 1.e9, 200.0; // GNSS latitude and longitude are not taken into account, we only correct altitude
            gnss_noise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // To DO: try different estimators
                        gtsam::noiseModel::Diagonal::Variances(gnss_cov) );
            // gnss_noise = gtsam::noiseModel::Diagonal::Variances(gnss_cov);

            gtsam::Vector loop_cov(6);
            loop_cov << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
            loop_noise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                        gtsam::noiseModel::Diagonal::Variances(loop_cov) );

            // ICP setup
            icp.setMaxCorrespondenceDistance(150);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Scan Context
            this->sc_ptr_ = std::make_unique<ScanContext>();

            this->initFlag = true;
        }

        bool Looper::solve(){
            if(not this->initFlag) return false;
            if(this->init_estimates.size() < 3 /*config.min_number_states*/) return false;

            // Check loop closure (To-DO: call loop check at different rate)
            this->check_loop();
            this->process_icp();

            this->graph_mtx.lock(); // avoid adding new state to the graph during iSAM update

            this->iSAM_->update(this->graph, this->init_estimates);
            this->iSAM_->update();

            // Compute estimate
            gtsam::Values result = this->iSAM_->calculateEstimate();
            this->out_estimate = result.at<gtsam::Pose3>(static_cast<int>(result.size())-1);

            // Update Key Frames clouds
            this->updateKeyFrames(&result);

            std::cout << "iSAM result size: " << result.size() << std::endl;

            // Compute marginals
            // gtsam::Marginals marginals(this->graph, result);
            // for(int i=0; i < result.size(); i++){
            //     std::cout << "x" << i << " covariance:\n" << marginals.marginalCovariance(i) << std::endl;
            // }
            
            std::cout << "------------------------ optimzed state -------------------------\n";
            std::cout << result.at<gtsam::Pose3>(static_cast<int>(result.size())-1) << std::endl;
            std::cout << "-----------------------------------------------------------------\n";

            this->graph.resize(0);
            this->init_estimates.clear();

            this->graph_mtx.unlock();

            return true;
        }

        void Looper::check_loop(){

            if(this->keyframes.size() < this->sc_ptr_->NUM_EXCLUDE_RECENT) return; // avoid checking too early

            auto lc_ids = sc_ptr_->detectLoopClosureID(); // loop closure indexes
            int closest_id = lc_ids.first;
            if(closest_id != -1){
                const int prev_idx = closest_id;
                const int curr_idx = this->keyframes.size()-1;

                // std::cout << "FAST_LIMO::LOOPER loop detected!!\n";
                // std::cout << "                  from: " << prev_idx << " to: " << curr_idx << std::endl;

                this->icp_mtx.lock();
                this->icp_candidates.push_front(std::make_pair(prev_idx, curr_idx));
                this->icp_mtx.unlock();
            }

        }

        void Looper::process_icp(){

            while(not this->icp_candidates.empty()){
                auto loop_idxs = icp_candidates.back();

                this->icp_mtx.lock();
                icp_candidates.pop_back();
                this->icp_mtx.unlock();

                auto pose_correction = run_icp(loop_idxs.first, loop_idxs.second);

                if( not pose_correction.equals(gtsam::Pose3::identity()) )
                    this->updateLoopClosure(pose_correction, loop_idxs.first, loop_idxs.second);
            }
        }

        void Looper::update(State s, pcl::PointCloud<PointType>::Ptr& pc){

            if(not time2update(s)) return;
            if(pc->points.size() <= 0) return;

            this->kf_mtx.lock();
            this->keyframes.push_front(std::make_pair(s, pc));
            this->kf_mtx.unlock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr sc_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*pc, *sc_cloud); // change cloud type

            pcl::PointCloud<pcl::PointXYZ>::Ptr bl_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // base link cloud
            pcl::transformPointCloud(*sc_cloud, *bl_cloud, s.get_RT_inv());

            sc_ptr_->makeAndSaveScanContextAndKeys(*bl_cloud);

            this->updateGraph(this->fromLIMOtoGTSAM(s));
        }

        void Looper::update(State s){
            if(not time2update(s)) return;
            this->updateGraph(this->fromLIMOtoGTSAM(s));
        }

        void Looper::update(double lat, double lng, double alt){

            if(not this->gnss_tf.hasLocalAxis()){
                gnss_tf.setInitPose(lat, lng, alt);
                return;
            } 

            if(this->init_estimates.size() < 1) return; // do not include GNSS if there aren't any betweenFactor estimates

            // Tranform from LLA to ENU
            Eigen::Vector3d ENU = gnss_tf.getENUpose(lat, lng, alt);
            if(not time2update(ENU)) return;

            double alt_offset = alt - gnss_tf.getInitPose()(2);
            gtsam::Pose3 last_estimate = init_estimates.at<gtsam::Pose3>(global_idx-1);
            gtsam::Point3 gnss_state(last_estimate.x(), last_estimate.y(), alt_offset);

            std::cout << "GNSS ALTITUDE offset: " << alt_offset << std::endl;

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

        std::vector<double> Looper::getPoseCovariance(){
            return std::vector<double>(36, 0.0);
        }

        std::vector<double> Looper::getTwistCovariance(){
            return std::vector<double>(36, 0.0);
        }

        std::vector< std::pair<State, 
                    pcl::PointCloud<PointType>::Ptr> > Looper::getKeyFrames(){

            std::vector<std::pair<State, pcl::PointCloud<PointType>::Ptr>> output;
            output.resize(this->keyframes.size());

            int num_threads = 10; // To-DO: make config param
            this->kf_mtx.lock();
            #pragma omp parallel for num_threads(num_threads)
            for(int i=0; i < this->keyframes.size(); i++){
                output[i] = this->keyframes[i];
            }
            this->kf_mtx.unlock();

            return output;
        }

    // private

        bool Looper::time2update(const State& s){ 
            /* To DO: 
                - pass gtsam::Pose as arg, use gtsam::between function for diff computation
            */
            Eigen::Vector3f    p_diff = s.p - last_kf.p;
            Eigen::Quaternionf q_diff = s.q * last_kf.q.inverse(); 

            Eigen::Vector3f rpy = q_diff.toRotationMatrix().eulerAngles(0, 1, 2);

            if(fabs(rpy(2)) > 0.5 || p_diff.norm() > 1.5){
                std::cout << "TIME TO UPDATE iSAM\n";
                this->last_kf = s;
                return true;
            }else
                return false;
        }

        bool Looper::time2update(Eigen::Vector3d& enu){
            Eigen::Vector3d p_diff = enu - this->last_enu;
            if(p_diff.norm() > 2.5){
                std::cout << "TIME TO UPDATE GNSS\n";
                this->last_enu = enu;
                return true;
            }else
                return false;
        }

        void Looper::updateGraph(gtsam::Pose3 pose){

            graph_mtx.lock();
            if(not this->priorAdded)
            {
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, this->prior_noise));
                init_estimates.insert(0, pose);
                priorAdded = true;
            }
            else
            {
                if(init_estimates.size() > 0)
                    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(global_idx-1, global_idx, 
                                                                init_estimates.at<gtsam::Pose3>(global_idx-1).between(pose), 
                                                                this->odom_noise) );
                else 
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

        void Looper::updateLoopClosure(gtsam::Pose3 pose, int idN, int id0){
            graph_mtx.lock();
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idN, id0, pose, this->loop_noise));
            graph_mtx.unlock();
        }

        gtsam::Pose3 Looper::fromLIMOtoGTSAM(const State& s){
            return gtsam::Pose3( gtsam::Rot3(gtsam::Rot3::Quaternion(s.q.w(), s.q.x(), s.q.y(), s.q.z())), 
                                 gtsam::Point3(s.p(0), s.p(1), s.p(2)) );
        }

        void Looper::updateKeyFrames(gtsam::Values* graph_estimate){
            
            this->kf_mtx.lock();

            auto kf_it = this->keyframes.begin();

            int idx = graph_estimate->size()-1;
            while( kf_it != this->keyframes.end() || idx > 0) {

                Eigen::Matrix4d Td = graph_estimate->at<gtsam::Pose3>(idx).matrix();
                fast_limo::State st(Td);

                Eigen::Matrix4f Tdiff = kf_it->first.get_RT_inv()*st.get_RT();

                // Update key pairs
                kf_it->first = st;

                pcl::PointCloud<PointType> updated_cloud;
                pcl::transformPointCloud (*(kf_it->second), updated_cloud, Tdiff);

                kf_it++;
                idx--;
            }

            this->kf_mtx.unlock();

        }

        gtsam::Pose3 Looper::run_icp(int id0, int idN){

            pcl::PointCloud<PointType>::Ptr currentKeyFrameCloud(this->keyframes[idN].second);
            pcl::PointCloud<PointType>::Ptr targetKeyFrameCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr correctedCloud(new pcl::PointCloud<PointType>());

            // Fill target cloud with near frames
            int kf_size = this->keyframes.size();
            for (int i = -20 /*config.window_size*/; i <= 25/*config.window_size*/; ++i)
            {
                int idx = id0 + i;
                if (idx < 0 || idx >= kf_size )
                    continue;

                this->kf_mtx.lock(); 
                *targetKeyFrameCloud += *this->keyframes[idx].second;
                this->kf_mtx.unlock();
            }

            // Align pointclouds
            this->icp.setInputSource(currentKeyFrameCloud);
            this->icp.setInputTarget(targetKeyFrameCloud);
            this->icp.align(*correctedCloud);

            if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3) {
                std::cout << "FAST_LIMO::LOOPER ICP correction failed (" << icp.getFitnessScore() << " > " << 0.3 << "). Rejecting this SC loop." << std::endl;
                return gtsam::Pose3::identity();
            } else {
                std::cout << "FAST_LIMO::LOOPER ICP fitness test passed (" << icp.getFitnessScore() << " < " << 0.3 << "). Adding this SC loop." << std::endl;
            }

            // Get pose transformation
            Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
            Eigen::Matrix4f current_tf = this->keyframes[idN].first.get_RT();
            Eigen::Matrix4f correct_tf = icp_tf * current_tf;

            gtsam::Pose3 poseFrom = this->fromLIMOtoGTSAM(fast_limo::State(correct_tf));
            gtsam::Pose3 poseTo = this->fromLIMOtoGTSAM(this->keyframes[id0].first);

            return poseFrom.between(poseTo);
        }