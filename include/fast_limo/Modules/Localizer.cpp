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

#include "fast_limo/Modules/Localizer.hpp"

// class fast_limo::Localizer
    // public

        Localizer::Localizer() : scan_stamp(0.0), prev_scan_stamp(0.0), scan_dt(0.1), deskew_size(0), propagated_size(0),
                                numProcessors(0), imu_stamp(0.0), prev_imu_stamp(0.0), imu_dt(0.005), first_imu_stamp(0.0),
                                last_propagate_time_(-1.0), imu_calib_time_(3.0), gravity_(9.81), imu_calibrated_(false)
                            { 

            this->original_scan  = pcl::PointCloud<PointType>::ConstPtr (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            this->deskewed_scan  = pcl::PointCloud<PointType>::ConstPtr (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            this->pc2match       = pcl::PointCloud<PointType>::Ptr (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            this->final_raw_scan = pcl::PointCloud<PointType>::Ptr (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            this->final_scan     = pcl::PointCloud<PointType>::Ptr (fast_limo::make_shared<pcl::PointCloud<PointType>>());
        }

        void Localizer::init(Config& cfg){

            // Save config
            this->config = cfg;

            // Set num of threads
            this->num_threads_ = omp_get_max_threads();
            if(num_threads_ > config.num_threads) this->num_threads_ = config.num_threads;

            // Update Mapper config
            fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
            map.set_num_threads(this->num_threads_);
            map.set_config(this->config.ikfom.mapping);

            // Initialize Iterated Kalman Filter on Manifolds
            this->init_iKFoM();

            // Set buffer capacity
            this->imu_buffer.set_capacity(2000);
            this->propagated_buffer.set_capacity(2000);

            // PCL filters setup
            this->crop_filter.setNegative(true);
            this->crop_filter.setMin(Eigen::Vector4f(config.filters.cropBoxMin[0], config.filters.cropBoxMin[1], config.filters.cropBoxMin[2], 1.0));
            this->crop_filter.setMax(Eigen::Vector4f(config.filters.cropBoxMax[0], config.filters.cropBoxMax[1], config.filters.cropBoxMax[2], 1.0));

            this->voxel_filter.setLeafSize(config.filters.leafSize[0], config.filters.leafSize[0], config.filters.leafSize[0]);

            // LiDAR sensor type
            this->set_sensor_type(config.sensor_type); 

            // IMU intrinsics
            this->imu_accel_sm_ = Eigen::Map<Eigen::Matrix3f>(config.intrinsics.imu_sm.data(), 3, 3);
            this->state.b.accel = Eigen::Map<Eigen::Vector3f>(config.intrinsics.accel_bias.data(), 3);
            this->state.b.gyro  = Eigen::Map<Eigen::Vector3f>(config.intrinsics.gyro_bias.data(), 3);

            // Extrinsics
            this->extr.imu2baselink.t = Eigen::Map<Eigen::Vector3f>(config.extrinsics.imu2baselink_t.data(), 3);
            Eigen::Matrix3f baselink2imu_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.imu2baselink_R.data(), 3, 3);
            this->extr.imu2baselink.R = baselink2imu_R.transpose();

            this->extr.imu2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.imu2baselink_T.block(0, 3, 3, 1) = this->extr.imu2baselink.t;
            this->extr.imu2baselink_T.block(0, 0, 3, 3) = this->extr.imu2baselink.R;

            this->extr.lidar2baselink.t = Eigen::Map<Eigen::Vector3f>(config.extrinsics.lidar2baselink_t.data(), 3);
            Eigen::Matrix3f baselink2lidar_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.lidar2baselink_R.data(), 3, 3);
            this->extr.lidar2baselink.R = baselink2lidar_R.transpose();

            this->extr.lidar2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.lidar2baselink_T.block(0, 3, 3, 1) = this->extr.lidar2baselink.t;
            this->extr.lidar2baselink_T.block(0, 0, 3, 3) = this->extr.lidar2baselink.R;

            // Avoid unnecessary warnings from PCL
            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

            // Initial calibration
            if( not (config.gravity_align || config.calibrate_accel || config.calibrate_gyro) ){ // no need for automatic calibration
                this->imu_calibrated_ = true;
                this->init_iKFoM_state();
            }

            // Calibration time
            this->imu_calib_time_ = config.imu_calib_time;

            // CPU info
            this->getCPUinfo();

            // Set up buffer capacities
            this->imu_rates.set_capacity(1000);
            this->lidar_rates.set_capacity(1000);
            this->cpu_times.set_capacity(1000);
            this->cpu_percents.set_capacity(1000);

            // Fill CPU stats
            this->cpu_time = 0.0f;
            this->cpu_max_time = 0.0f;
            this->cpu_mean_time = 0.0f;
            this->cpu_cores = 0.0f;
            this->cpu_load = 0.0f;
            this->cpu_max_load = 0.0f;
            this->ram_usage = 0.0f;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_pointcloud(){
            return this->final_scan;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_finalraw_pointcloud(){
            return this->final_raw_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_orig_pointcloud(){
            return this->original_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_deskewed_pointcloud(){
            return this->deskewed_scan;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_pc2match_pointcloud(){
            return this->pc2match;
        }

        Matches& Localizer::get_matches(){
            return this->matches;
        }

        bool Localizer::is_calibrated(){
            return this->imu_calibrated_;
        }

        void Localizer::set_sensor_type(uint8_t type){
            if(type < 5)
                this->sensor = static_cast<fast_limo::SensorType>(type);
            else 
                this->sensor = fast_limo::SensorType::UNKNOWN;
        }

        fast_limo::SensorType Localizer::get_sensor_type(){
            return this->sensor;
        }

        State Localizer::getBodyState(){

            if(not this->is_calibrated())
                return State();

            State out = this->_iKFoM.get_x();

            out.w    = this->last_imu.ang_vel;                      // set last IMU meas
            out.a    = this->last_imu.lin_accel;                    // set last IMU meas
            out.time = this->imu_stamp;                             // set current time stamp 

            out.p += out.pLI;                                       // position in LiDAR frame
            out.q *= out.qLI;                                       // attitude in LiDAR frame
            out.v = out.q.toRotationMatrix().transpose() * out.v;   // local velocity vector
            return out;
        }

        State Localizer::getWorldState(){

            if(not this->is_calibrated())
                return State();

            State out = this->_iKFoM.get_x();

            out.w    = this->last_imu.ang_vel;                      // set last IMU meas
            out.a    = this->last_imu.lin_accel;                    // set last IMU meas
            out.time = this->imu_stamp;                             // set current time stamp 

            out.v = out.q.toRotationMatrix().transpose() * out.v;   // local velocity vector

            return out;
        }

        double Localizer::get_propagate_time(){
            return this->last_propagate_time_;
        }

        void Localizer::get_cpu_stats(float &comput_time, float &max_comput_time, float &mean_comput_time,
                            float &cpu_cores, float &cpu_load, float &cpu_max_load, float &ram_usage){
            
            this->mtx_cpu_stats.lock();
            comput_time = this->cpu_time;
            max_comput_time = this->cpu_max_time;
            mean_comput_time = this->cpu_mean_time;
            cpu_cores = this->cpu_cores;
            cpu_load = this->cpu_load;
            cpu_max_load = this->cpu_max_load;
            ram_usage = this->ram_usage;
            this->mtx_cpu_stats.unlock();
        }

        std::vector<double> Localizer::getPoseCovariance(){
            if(not this->is_calibrated())
                return std::vector<double>(36, 0);

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P = this->_iKFoM.get_P();
            Eigen::Matrix<double, 6, 6> P_pose;
            P_pose.block<3, 3>(0, 0) = P.block<3, 3>(3, 3);
            P_pose.block<3, 3>(0, 3) = P.block<3, 3>(3, 0);
            P_pose.block<3, 3>(3, 0) = P.block<3, 3>(0, 3);
            P_pose.block<3, 3>(3, 3) = P.block<3, 3>(0, 0);

            std::vector<double> cov(P_pose.size());
            Eigen::Map<Eigen::MatrixXd>(cov.data(), P_pose.rows(), P_pose.cols()) = P_pose;

            return cov;
        }

        std::vector<double> Localizer::getTwistCovariance(){
            if(not this->is_calibrated())
                return std::vector<double>(36, 0);

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P = this->_iKFoM.get_P();
            Eigen::Matrix<double, 6, 6> P_odom = Eigen::Matrix<double, 6, 6>::Zero();
            P_odom.block<3, 3>(0, 0) = P.block<3, 3>(6, 6);
            P_odom.block<3, 3>(3, 3) = config.ikfom.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();

            std::vector<double> cov(P_odom.size());
            Eigen::Map<Eigen::MatrixXd>(cov.data(), P_odom.rows(), P_odom.cols()) = P_odom;

            return cov;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////           Principal callbacks/threads        /////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp){

            auto start_time = std::chrono::system_clock::now();

            if(raw_pc->points.size() < 1){
                std::cout << "FAST_LIMO::Raw PointCloud is empty!\n";
                return;
            }

            if(!this->imu_calibrated_)
                return;

            if(this->imu_buffer.empty()){
                std::cout << "FAST_LIMO::IMU buffer is empty!\n";
                return;
            }

            // Remove NaNs
            std::vector<int> idx;
            raw_pc->is_dense = false;
            pcl::removeNaNFromPointCloud(*raw_pc, *raw_pc, idx);

            // Crop Box Filter (1 m^2)
            if(this->config.filters.crop_active){
                this->crop_filter.setInputCloud(raw_pc);
                this->crop_filter.filter(*raw_pc);
            }

            // Distance & Time Rate filters
            static float min_dist = static_cast<float>(this->config.filters.min_dist);
            static int rate_value = this->config.filters.rate_value;
            std::function<bool(boost::range::index_value<PointType&, long>)> filter_f;
            
            if(this->config.filters.dist_active && this->config.filters.rate_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist)
                                && (p.index()%rate_value == 0) && this->isInRange(p.value()); };
            }
            else if(this->config.filters.dist_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist) &&
                                this->isInRange(p.value()); };
            }
            else if(this->config.filters.rate_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (p.index()%rate_value == 0) && this->isInRange(p.value()); };
            }else{
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return this->isInRange(p.value()); };
            }
            auto filtered_pc = raw_pc->points 
                        | boost::adaptors::indexed()
                        | boost::adaptors::filtered(filter_f);

            pcl::PointCloud<PointType>::Ptr input_pc (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            for (auto it = filtered_pc.begin(); it != filtered_pc.end(); it++) {
                input_pc->points.push_back(it->value());
            }

            if(this->config.debug) // debug only
                this->original_scan = fast_limo::make_shared<pcl::PointCloud<PointType>>(*input_pc); // LiDAR frame

            // Motion compensation
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_pc_ (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_pc_ = this->deskewPointCloud(input_pc, time_stamp);
            /*NOTE: deskewed_Xt2_pc_ should be in base_link/body frame w.r.t last propagated state (Xt2) */

            // Voxel Grid Filter
            if (this->config.filters.voxel_active) { 
                pcl::PointCloud<PointType>::Ptr current_scan_
                    (fast_limo::make_shared<pcl::PointCloud<PointType>>(*deskewed_Xt2_pc_));
                this->voxel_filter.setInputCloud(current_scan_);
                this->voxel_filter.filter(*current_scan_);
                this->pc2match = current_scan_;
            } else {
                this->pc2match = deskewed_Xt2_pc_;
            }

            if(this->pc2match->points.size() > 1){

                // iKFoM observation stage
                this->mtx_ikfom.lock();

                // Call Mapper obj
                fast_limo::Mapper& map = fast_limo::Mapper::getInstance();

                    // Update iKFoM measurements (after prediction)
                double solve_time = 0.0;
                this->_iKFoM.update_iterated_dyn_share_modified(0.001 /*LiDAR noise*/, 5.0/*Degeneracy threshold*/, 
                                                                solve_time/*solving time elapsed*/, false/*print degeneracy values flag*/);
                    /*NOTE: update_iterated_dyn_share_modified() will trigger the matching procedure ( see "use-ikfom.cpp" )
                    in order to update the measurement stage of the KF with the computed point-to-plane distances*/
                
                map.matches.clear(); // clear matches vector for next iteration

                    // Get output state from iKFoM
                fast_limo::State corrected_state = fast_limo::State(this->_iKFoM.get_x());

                // Set estimated biases & gravity to constant
                if(this->config.calibrate_gyro)  corrected_state.b.gyro  = this->state.b.gyro;
                if(this->config.calibrate_accel) corrected_state.b.accel = this->state.b.accel;
                if(this->config.gravity_align)   corrected_state.g       = this->state.g;

                // Update current state estimate
                this->state      = corrected_state;
                this->state.w    = this->last_imu.ang_vel;
                this->state.a    = this->last_imu.lin_accel;

                this->mtx_ikfom.unlock();

                // Get estimated offset
                this->extr.lidar2baselink_T = this->state.get_extr_RT();

                // Transform deskewed pc 
                    // Get deskewed scan to add to map
                pcl::PointCloud<PointType>::Ptr mapped_scan (fast_limo::make_shared<pcl::PointCloud<PointType>>());
                mapped_scan->points.resize(pc2match->points.size());

                // pcl::transformPointCloud (*this->pc2match, *mapped_scan, this->state.get_RT()); // Not working for PCL 1.12
                #pragma omp parallel for num_threads(this->num_threads_)
                for(int i=0; i<pc2match->points.size(); i++){
                    PointType pt = pc2match->points[i];
                    pt.getVector4fMap()[3] = 1.;
                    pt.getVector4fMap() = this->state.get_RT() * pt.getVector4fMap(); 
                    mapped_scan->points[i] = pt;
                    /*NOTE: pc2match must be in base_link frame w.r.t Xt2 frame for this transform to work.
                        mapped_scan is in world/global frame.
                    */
                }

                /*To DO:
                    - mapped_scan --> segmentation of dynamic objects
                */

                    // Get final scan to output (in world/global frame)
                this->final_scan = mapped_scan; // mapped_scan = final_scan (for now)
                // pcl::transformPointCloud (*this->pc2match, *this->final_scan, this->state.get_RT()); // Not working for PCL 1.12

                if(this->config.debug){ // save final scan without voxel grid
                    final_raw_scan->points.clear();
                    final_raw_scan->points.resize(deskewed_Xt2_pc_->points.size());
                    #pragma omp parallel for num_threads(this->num_threads_)
                    for(int i=0; i<deskewed_Xt2_pc_->points.size(); i++){
                        PointType pt = deskewed_Xt2_pc_->points[i];
                        pt.getVector4fMap()[3] = 1.;
                        pt.getVector4fMap() = this->state.get_RT() * pt.getVector4fMap(); 
                        final_raw_scan->points[i] = pt;
                    }
                    // pcl::transformPointCloud (*deskewed_Xt2_pc_, *this->final_raw_scan, this->state.get_RT()); // Not working for PCL 1.12
                }

                // Add scan to map
                map.add(mapped_scan, this->scan_stamp);

            }else
                std::cout << "-------------- FAST_LIMO::NULL ITERATION --------------\n";

            auto end_time = std::chrono::system_clock::now();
            elapsed_time = end_time - start_time;

            // fill stats
            if(this->prev_scan_stamp > 0.0) this->lidar_rates.push_front( 1. / (this->scan_stamp - this->prev_scan_stamp) );
            if(calibrating > 0) this->cpu_times.push_front(elapsed_time.count());
            else{
                this->cpu_times.push_front(0.0);
                calibrating++;
            }
            // if(calibrating < UCHAR_MAX) calibrating++;

            // debug thread
            this->debug_thread = std::thread( &Localizer::debugVerbose, this );
            this->debug_thread.detach();

            this->prev_scan_stamp = this->scan_stamp;
        }

        void Localizer::updateIMU(IMUmeas& raw_imu){

            this->imu_stamp = raw_imu.stamp;
            IMUmeas imu = this->imu2baselink(raw_imu);

            if(this->first_imu_stamp == 0.0)
                this->first_imu_stamp = imu.stamp;

            this->imu_rates.push_front( 1./imu.dt );

            // IMU calibration procedure - do only while the robot is in stand still!
            if (not this->imu_calibrated_) {

                static int num_samples = 0;
                static Eigen::Vector3f gyro_avg (0., 0., 0.);
                static Eigen::Vector3f accel_avg (0., 0., 0.);
                static bool print = true;

                if ((imu.stamp - this->first_imu_stamp) < this->imu_calib_time_) {

                    num_samples++;

                    gyro_avg[0] += imu.ang_vel[0];
                    gyro_avg[1] += imu.ang_vel[1];
                    gyro_avg[2] += imu.ang_vel[2];

                    accel_avg[0] += imu.lin_accel[0];
                    accel_avg[1] += imu.lin_accel[1];
                    accel_avg[2] += imu.lin_accel[2];

                    if(print) {
                        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... \n";
                        std::cout.flush();
                        print = false;
                    }

                } else {

                    gyro_avg /= num_samples;
                    accel_avg /= num_samples;

                    Eigen::Vector3f grav_vec (0., 0., this->gravity_);

                    this->state.q = imu.q;

                    if (this->config.gravity_align) {

                        std::cout << " Accel mean: " << "[ " << accel_avg[0] << ", " << accel_avg[1] << ", " << accel_avg[2] << " ]\n";

                        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
                        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));
                        
                        std::cout << " Gravity mean: " << "[ " << grav_vec[0] << ", " << grav_vec[1] << ", " << grav_vec[2] << " ]\n";

                        // set gravity aligned orientation
                        this->state.q = grav_q;

                        // set estimated gravity vector
                        this->state.g = grav_vec;

                    }

                    if (this->config.calibrate_accel) {

                        // subtract gravity from avg accel to get bias
                        this->state.b.accel = accel_avg - grav_vec;

                        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
                    }

                    if (this->config.calibrate_gyro) {

                        this->state.b.gyro = gyro_avg;

                        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
                    }

                    this->state.q.normalize();

                    // Set initial KF state
                    this->init_iKFoM_state();

                    // Set calib flag
                    this->imu_calibrated_ = true;

                    // Initial attitude
                    auto euler = this->state.q.toRotationMatrix().eulerAngles(2, 1, 0);
                    double yaw = euler[0] * (180.0/M_PI);
                    double pitch = euler[1] * (180.0/M_PI);
                    double roll = euler[2] * (180.0/M_PI);

                    // use alternate representation if the yaw is smaller
                    if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                        yaw   = remainder(yaw + 180.0,   360.0);
                        pitch = remainder(180.0 - pitch, 360.0);
                        roll  = remainder(roll + 180.0,  360.0);
                    }
                    std::cout << " Estimated initial attitude:" << std::endl;
                    std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
                    std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
                    std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
                    std::cout << std::endl;

                }

            } else {

                // Apply the calibrated bias to the new IMU measurements
                Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * imu.lin_accel) - this->state.b.accel;
                Eigen::Vector3f ang_vel_corrected = imu.ang_vel - this->state.b.gyro;

                imu.lin_accel = lin_accel_corrected;
                imu.ang_vel   = ang_vel_corrected;

                this->last_imu = imu;

                // Store calibrated IMU measurements into imu buffer for manual integration later.
                this->imu_buffer.push_front(imu);

                // iKFoM propagate state
                this->propagateImu(imu);
                this->cv_prop_stamp.notify_one(); // Notify PointCloud thread that propagated IMU data exists for this time
            }

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          KF measurement model        /////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::calculate_H(const state_ikfom& s, const Matches& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h){
            
            int N = (matches.size() > config.ikfom.mapping.MAX_NUM_MATCHES) ? config.ikfom.mapping.MAX_NUM_MATCHES : matches.size();

            H = Eigen::MatrixXd::Zero(N, 12);
            h.resize(N);
            State S(s);

            // For each match, calculate its derivative and distance
            #pragma omp parallel for num_threads(this->num_threads_)
            for (int i = 0; i < N; ++i) {
                Match match = matches[i];
                Eigen::Vector4f p4_imu   = S.get_RT_inv() /*world2baselink*/ * match.get_4Dglobal();
                Eigen::Vector4f p4_lidar = S.get_extr_RT_inv() /* baselink2lidar */ * p4_imu;
                Eigen::Vector4f normal   = match.plane.get_normal();

                // Rotation matrices
                Eigen::Matrix3f R_inv = s.rot.conjugate().toRotationMatrix().cast<float>();
                Eigen::Matrix3f I_R_L_inv = s.offset_R_L_I.conjugate().toRotationMatrix().cast<float>();

                // Set correct dimensions
                Eigen::Vector3f p_lidar, p_imu, n;
                p_lidar = p4_lidar.head(3);
                p_imu   = p4_imu.head(3);
                n       = normal.head(3);

                // Calculate measurement Jacobian H (:= dh/dx)
                Eigen::Vector3f C = R_inv * n;
                Eigen::Vector3f B = p_lidar.cross(I_R_L_inv * C);
                Eigen::Vector3f A = p_imu.cross(C);
                
                H.block<1, 6>(i,0) << n(0), n(1), n(2), A(0), A(1), A(2);
                if (config.ikfom.estimate_extrinsics) H.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

                // Measurement: distance to the closest plane
                h(i) = -match.dist;
            }

            if(this->config.debug) 
                this->matches = matches;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          KF propagation model        /////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::propagateImu(const IMUmeas& imu){
            input_ikfom in;
            in.acc = imu.lin_accel.cast<double>();
            in.gyro = imu.ang_vel.cast<double>();

            Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
            Q.block<3, 3>(0, 0) = config.ikfom.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(3, 3) = config.ikfom.cov_acc * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(6, 6) = config.ikfom.cov_bias_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(9, 9) = config.ikfom.cov_bias_acc * Eigen::Matrix<double, 3, 3>::Identity();

            // Propagate IMU measurement
            double dt = imu.dt;
            this->mtx_ikfom.lock();
            this->_iKFoM.predict(dt, Q, in);
            this->mtx_ikfom.unlock();

            // Save propagated state for motion compensation
            this->mtx_prop.lock();
            this->propagated_buffer.push_front( fast_limo::State(this->_iKFoM.get_x(), 
                                                                imu.stamp, imu.lin_accel, imu.ang_vel)
                                                );
            this->mtx_prop.unlock();

            this->last_propagate_time_ = imu.stamp;
        }

        void Localizer::propagateImu(double t1, double t2){

            Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
            Q.block<3, 3>(0, 0) = config.ikfom.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(3, 3) = config.ikfom.cov_acc * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(6, 6) = config.ikfom.cov_bias_gyro * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(9, 9) = config.ikfom.cov_bias_acc * Eigen::Matrix<double, 3, 3>::Identity();

            boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it;
            boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it;
            if (not this->imuMeasFromTimeRange(t1, t2, begin_imu_it, end_imu_it)) {
                // not enough IMU measurements, return empty vector
                std::cout << "FAST_LIMO::propagateImu(): not enough IMU measurements\n";
                return;
            }

            // Iterate over IMU measurements
            auto imu_it = begin_imu_it;

            // Propagate IMU meas and save it for motion compensation
            this->mtx_ikfom.lock();
            this->mtx_prop.lock();

            input_ikfom input;
            double dt;
            for (; imu_it != end_imu_it; imu_it++) {
                const IMUmeas& imu = *imu_it;

                input.acc  = imu.lin_accel.cast<double>();
                input.gyro = imu.ang_vel.cast<double>();
                dt = imu.dt;

                this->_iKFoM.predict(dt, Q, input);
                this->propagated_buffer.push_front( fast_limo::State(this->_iKFoM.get_x(), 
                                                                imu.stamp, imu.lin_accel, imu.ang_vel)
                                                    );
            }

            this->mtx_ikfom.unlock();
            this->mtx_prop.unlock();

            this->last_propagate_time_ = end_imu_it->stamp;
        }

    // private

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          Aux. functions        ///////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::init_iKFoM() {
            // Initialize IKFoM
            this->_iKFoM.init_dyn_share(
                IKFoM::get_f,
                IKFoM::df_dx,
                IKFoM::df_dw,
                IKFoM::h_share_model,
                config.ikfom.MAX_NUM_ITERS,
                config.ikfom.LIMITS
            );
        }

        void Localizer::init_iKFoM_state() {
            state_ikfom init_state = this->_iKFoM.get_x();
            init_state.rot = this->state.q.cast<double> ();
            init_state.pos = this->state.p.cast<double> ();
            init_state.grav = /*MTK::*/S2(Eigen::Vector3d(0., 0., -this->gravity_));
            init_state.bg = this->state.b.gyro.cast<double>();
            init_state.ba = this->state.b.accel.cast<double>();

            // set up offsets (LiDAR -> BaseLink transform == LiDAR pose w.r.t. BaseLink)
            init_state.offset_R_L_I = /*MTK::*/SO3(this->extr.lidar2baselink.R.cast<double>());
            init_state.offset_T_L_I = this->extr.lidar2baselink.t.cast<double>();
            this->_iKFoM.change_x(init_state); // set initial state

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = this->_iKFoM.get_P();
            init_P.setIdentity();
            init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.000001;
            init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.000001;
            init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.00001;
            init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.0001;
            init_P(21,21) = init_P(22,22) = 0.000001; 
            
            this->_iKFoM.change_P(init_P);
        }

        IMUmeas Localizer::imu2baselink(IMUmeas& imu){

            IMUmeas imu_baselink;

            double dt = imu.stamp - this->prev_imu_stamp;
            
            if ( (dt == 0.) || (dt > 0.1) ) { dt = 1.0/200.0; }

            // Transform angular velocity (will be the same on a rigid body, so just rotate to baselink frame)
            Eigen::Vector3f ang_vel_cg = this->extr.imu2baselink.R * imu.ang_vel;

            static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

            // Transform linear acceleration (need to account for component due to translational difference)
            Eigen::Vector3f lin_accel_cg = this->extr.imu2baselink.R * imu.lin_accel;

            lin_accel_cg = lin_accel_cg
                            + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extr.imu2baselink.t)
                            + ang_vel_cg.cross(ang_vel_cg.cross(-this->extr.imu2baselink.t));

            ang_vel_cg_prev = ang_vel_cg;

            imu_baselink.ang_vel   = ang_vel_cg;
            imu_baselink.lin_accel = lin_accel_cg;
            imu_baselink.dt        = dt;
            imu_baselink.stamp     = imu.stamp;

            Eigen::Quaternionf q(this->extr.imu2baselink.R);
            q.normalize();
            imu_baselink.q = q * imu.q;

            this->prev_imu_stamp = imu.stamp;

            return imu_baselink;

        }

        pcl::PointCloud<PointType>::Ptr
        Localizer::deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time){

            if(pc->points.size() < 1) 
                return fast_limo::make_shared<pcl::PointCloud<PointType>>();

            // individual point timestamps should be relative to this time
            double sweep_ref_time = start_time;
            bool end_of_sweep = this->config.end_of_sweep;

            // sort points by timestamp
            std::function<bool(const PointType&, const PointType&)> point_time_cmp;
            std::function<double(PointType&)> extract_point_time;

            if (this->sensor == fast_limo::SensorType::OUSTER) {

                point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
                {   if (end_of_sweep) return p1.t > p2.t; 
                    else return p1.t < p2.t; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.t * 1e-9f; 
                    else return sweep_ref_time + pt.t * 1e-9f; };

            } else if (this->sensor == fast_limo::SensorType::VELODYNE) {
                
                point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
                {   if (end_of_sweep) return p1.time > p2.time; 
                    else return p1.time < p2.time; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.time; 
                    else return sweep_ref_time + pt.time; };

            } else if (this->sensor == fast_limo::SensorType::HESAI) {

                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                extract_point_time = [](PointType& pt)
                { return pt.timestamp; };

            } else if (this->sensor == fast_limo::SensorType::LIVOX) {
                
                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.timestamp * 1e-9f; 
                    else return sweep_ref_time + pt.timestamp * 1e-9f; };
            } else {
                std::cout << "-------------------------------------------------------------------\n";
                std::cout << "FAST_LIMO::FATAL ERROR: LiDAR sensor type unknown or not specified!\n";
                std::cout << "-------------------------------------------------------------------\n";
                return fast_limo::make_shared<pcl::PointCloud<PointType>>();
            }

            // copy points into deskewed_scan_ in order of timestamp
            pcl::PointCloud<PointType>::Ptr deskewed_scan_ (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            deskewed_scan_->points.resize(pc->points.size());
            
            std::partial_sort_copy(pc->points.begin(), pc->points.end(),
                                    deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

            if(deskewed_scan_->points.size() < 1){
                std::cout << "FAST_LIMO::ERROR: failed to sort input pointcloud!\n";
                return fast_limo::make_shared<pcl::PointCloud<PointType>>();
            }

            // compute offset between sweep reference time and IMU data
            double offset = 0.0;
            if (config.time_offset) {
                offset = this->imu_stamp - extract_point_time(deskewed_scan_->points[deskewed_scan_->points.size()-1]) - 1.e-4; // automatic sync (not precise!)
                if(offset > 0.0) offset = 0.0; // don't jump into future
            }

            // Set scan_stamp for next iteration
            this->scan_stamp = extract_point_time(deskewed_scan_->points[deskewed_scan_->points.size()-1]) + offset;

            // IMU prior & deskewing 
            States frames = this->integrateImu(this->prev_scan_stamp, this->scan_stamp, this->state); // baselink/body frames

            if(frames.size() < 1){
                std::cout << "FAST_LIMO::ERROR: No frames obtained from IMU propagation!\n";
                std::cout << "           Returning null deskewed pointcloud!\n";
                return fast_limo::make_shared<pcl::PointCloud<PointType>>();
            }

            // deskewed pointcloud w.r.t last known state prediction
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_scan_ (fast_limo::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_scan_->points.resize(deskewed_scan_->points.size());

            this->last_state = fast_limo::State(this->_iKFoM.get_x()); // baselink/body frame

            #pragma omp parallel for num_threads(this->num_threads_)
            for (int k = 0; k < deskewed_scan_->points.size(); k++) {

                int i_f = algorithms::binary_search_tailored(frames, extract_point_time(deskewed_scan_->points[k])+offset);

                State X0 = frames[i_f];
                X0.update(extract_point_time(deskewed_scan_->points[k]) + offset);

                Eigen::Matrix4f T = X0.get_RT() * this->extr.lidar2baselink_T;

                // world frame deskewed pc
                auto &pt = deskewed_scan_->points[k]; // lidar frame
                pt.getVector4fMap()[3] = 1.;
                pt.getVector4fMap() = T * pt.getVector4fMap(); // world/global frame

                // Xt2 frame deskewed pc
                auto pt2 = deskewed_scan_->points[k];
                pt2.getVector4fMap() = this->last_state.get_RT_inv() * pt.getVector4fMap(); // Xt2 frame
                pt2.intensity = pt.intensity;

                deskewed_Xt2_scan_->points[k] = pt2;
            }

            // debug info
            this->deskew_size = deskewed_Xt2_scan_->points.size(); 
            this->propagated_size = frames.size();

            if(this->config.debug && this->deskew_size > 0) // debug only
                this->deskewed_scan = deskewed_scan_;

            return deskewed_Xt2_scan_; 
        }

        States Localizer::integrateImu(double start_time, double end_time, State& state){

            States imu_se3;

            boost::circular_buffer<State>::reverse_iterator begin_prop_it;
            boost::circular_buffer<State>::reverse_iterator end_prop_it;
            if (not this->propagatedFromTimeRange(start_time, end_time, begin_prop_it, end_prop_it)) {
                // not enough IMU measurements, return empty vector
                std::cout << "FAST_LIMO::propagatedFromTimeRange(): not enough propagated states!\n";
                return imu_se3;
            }

            for(auto it = begin_prop_it; it != end_prop_it; it++)
                imu_se3.push_back(*it);

            return imu_se3;
        }

        bool Localizer::isInRange(PointType& p){
            if(not this->config.filters.fov_active) return true;
            return fabs(atan2(p.y, p.x)) < this->config.filters.fov_angle;
        }

        bool Localizer::propagatedFromTimeRange(double start_time, double end_time,
                                                boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
                                                boost::circular_buffer<State>::reverse_iterator& end_prop_it) {

            if (this->propagated_buffer.empty() || this->propagated_buffer.front().time < end_time) {
                // Wait for the latest IMU data
                std::cout << "PROPAGATE WAITING...\n";
                std::cout << "     - buffer time: " << std::setprecision(15) << propagated_buffer.front().time << std::endl;
                std::cout << "     - end scan time: " << std::setprecision(15) << end_time << std::endl;
                std::unique_lock<decltype(this->mtx_prop)> lock(this->mtx_prop);
                this->cv_prop_stamp.wait(lock, [this, &end_time]{ return this->propagated_buffer.front().time >= end_time; });
            }

            auto prop_it = this->propagated_buffer.begin();

            auto last_prop_it = prop_it;
            prop_it++;
            while (prop_it != this->propagated_buffer.end() && prop_it->time >= end_time) {
                last_prop_it = prop_it;
                prop_it++;
            }

            while (prop_it != this->propagated_buffer.end() && prop_it->time >= start_time) {
                prop_it++;
            }

            if (prop_it == this->propagated_buffer.end()) {
                // not enough IMU measurements, return false
                return false;
            }
            prop_it++;

            // Set reverse iterators (to iterate forward in time)
            end_prop_it = boost::circular_buffer<State>::reverse_iterator(last_prop_it);
            begin_prop_it = boost::circular_buffer<State>::reverse_iterator(prop_it);

            return true;
        }

        bool Localizer::imuMeasFromTimeRange(double start_time, double end_time,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it) {

            if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
                return false;
            }

            auto imu_it = this->imu_buffer.begin();

            auto last_imu_it = imu_it;
            imu_it++;
            while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
                last_imu_it = imu_it;
                imu_it++;
            }

            while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
                imu_it++;
            }

            if (imu_it == this->imu_buffer.end()) {
                // not enough IMU measurements, return false
                return false;
            }
            imu_it++;

            // Set reverse iterators (to iterate forward in time)
            end_imu_it = boost::circular_buffer<IMUmeas>::reverse_iterator(last_imu_it);
            begin_imu_it = boost::circular_buffer<IMUmeas>::reverse_iterator(imu_it);

            return true;
        }

        void Localizer::getCPUinfo(){ // CPU Specs
            char CPUBrandString[0x40];
            memset(CPUBrandString, 0, sizeof(CPUBrandString));

            this->cpu_type = "";

            #ifdef HAS_CPUID
            unsigned int CPUInfo[4] = {0,0,0,0};
            __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
            unsigned int nExIds = CPUInfo[0];
            for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
                __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
                if (i == 0x80000002)
                memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
                else if (i == 0x80000003)
                memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
                else if (i == 0x80000004)
                memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
            }
            this->cpu_type = CPUBrandString;
            boost::trim(this->cpu_type);
            #endif

            FILE* file;
            struct tms timeSample;
            char line[128];

            this->lastCPU = times(&timeSample);
            this->lastSysCPU = timeSample.tms_stime;
            this->lastUserCPU = timeSample.tms_utime;

            file = fopen("/proc/cpuinfo", "r");
            this->numProcessors = 0;
            while(fgets(line, 128, file) != nullptr) {
                if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
            }
            fclose(file);
        }

        void Localizer::debugVerbose(){

            // Average computation time
            double avg_comp_time =
                std::accumulate(this->cpu_times.begin(), this->cpu_times.end(), 0.0) / this->cpu_times.size();

            // Average sensor rates
            double avg_imu_rate = (this->imu_rates.size() > 0) ?
                std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size() : 0.0;
            double avg_lidar_rate = (this->lidar_rates.size() > 0) ?
                std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size() : 0.0;

            // RAM Usage
            double vm_usage = 0.0;
            double resident_set = 0.0;
            std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
            std::string pid, comm, state, ppid, pgrp, session, tty_nr;
            std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
            std::string utime, stime, cutime, cstime, priority, nice;
            std::string num_threads, itrealvalue, starttime;
            unsigned long vsize;
            long rss;
            stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                        >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                        >> utime >> stime >> cutime >> cstime >> priority >> nice
                        >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
            stat_stream.close();
            long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
            vm_usage = vsize / 1024.0;
            resident_set = rss * page_size_kb;

            // CPU Usage
            struct tms timeSample;
            clock_t now;
            double cpu_percent;
            now = times(&timeSample);
            if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
                timeSample.tms_utime < this->lastUserCPU) {
                cpu_percent = -1.0;
            } else {
                cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
                cpu_percent /= (now - this->lastCPU);
                cpu_percent /= this->numProcessors;
                cpu_percent *= 100.;
            }
            this->lastCPU = now;
            this->lastSysCPU = timeSample.tms_stime;
            this->lastUserCPU = timeSample.tms_utime;
            this->cpu_percents.push_front(cpu_percent);
            double avg_cpu_usage =
                std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

            // ------------------------------------- PRINT OUT -------------------------------------

            if(this->config.verbose){

            printf("\033[2J\033[1;1H");
            std::cout << std::endl
                        << "+-------------------------------------------------------------------+" << std::endl;
            std::cout   << "|                        Fast LIMO  v" << FAST_LIMO_v  << "                          |"
                        << std::endl;
            std::cout   << "+-------------------------------------------------------------------+" << std::endl;

            std::time_t curr_time = this->scan_stamp;
            std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
            std::cout << "| " << std::left << asc_time;
            std::cout << std::right << std::setfill(' ') << std::setw(42)
                << "Elapsed Time: " + to_string_with_precision(this->imu_stamp - this->first_imu_stamp, 2) + " seconds "
                << "|" << std::endl;

            if ( !this->cpu_type.empty() ) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << this->cpu_type + " x " + std::to_string(this->numProcessors)
                << "|" << std::endl;
            }

            if (this->sensor == fast_limo::SensorType::OUSTER) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == fast_limo::SensorType::VELODYNE) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
                                                + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == fast_limo::SensorType::HESAI) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == fast_limo::SensorType::LIVOX) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                                    + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            }

            std::cout << "|===================================================================|" << std::endl;

            State final_state = this->getWorldState();

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Position     {W}  [xyz] :: " + to_string_with_precision(final_state.p(0), 4) + " "
                                            + to_string_with_precision(final_state.p(1), 4) + " "
                                            + to_string_with_precision(final_state.p(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Orientation  {W} [wxyz] :: " + to_string_with_precision(final_state.q.w(), 4) + " "
                                            + to_string_with_precision(final_state.q.x(), 4) + " "
                                            + to_string_with_precision(final_state.q.y(), 4) + " "
                                            + to_string_with_precision(final_state.q.z(), 4)
                << "|" << std::endl;

            auto euler = final_state.q.toRotationMatrix().eulerAngles(2, 1, 0);
            double yaw = euler[0] * (180.0/M_PI);
            double pitch = euler[1] * (180.0/M_PI);
            double roll = euler[2] * (180.0/M_PI);

            // use alternate representation if the yaw is smaller
            if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
            }
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "             {W} [ypr] :: " + to_string_with_precision(yaw, 4) + " "
                                            + to_string_with_precision(pitch, 4) + " "
                                            + to_string_with_precision(roll, 4)
                << "|" << std::endl;

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(final_state.v(0), 4) + " "
                                            + to_string_with_precision(final_state.v(1), 4) + " "
                                            + to_string_with_precision(final_state.v(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(final_state.w(0), 4) + " "
                                            + to_string_with_precision(final_state.w(1), 4) + " "
                                            + to_string_with_precision(final_state.w(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Accel Bias        [xyz] :: " + to_string_with_precision(final_state.b.accel(0), 8) + " "
                                            + to_string_with_precision(final_state.b.accel(1), 8) + " "
                                            + to_string_with_precision(final_state.b.accel(2), 8)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Gyro Bias         [xyz] :: " + to_string_with_precision(final_state.b.gyro(0), 8) + " "
                                            + to_string_with_precision(final_state.b.gyro(1), 8) + " "
                                            + to_string_with_precision(final_state.b.gyro(2), 8)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Gravity Est.      [xyz] :: " + to_string_with_precision(final_state.g(0), 8) + " "
                                            + to_string_with_precision(final_state.g(1), 8) + " "
                                            + to_string_with_precision(final_state.g(2), 8)
                << "|" << std::endl;

            std::cout << "|                                                                   |" << std::endl;


            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "LiDAR -> BaseLink     [t] :: " + to_string_with_precision(final_state.pLI(0), 4) + " "
                                            + to_string_with_precision(final_state.pLI(1), 4) + " "
                                            + to_string_with_precision(final_state.pLI(2), 4)
                << "|" << std::endl;
            
            euler = final_state.qLI.toRotationMatrix().eulerAngles(2, 1, 0);
            yaw = euler[0] * (180.0/M_PI);
            pitch = euler[1] * (180.0/M_PI);
            roll = euler[2] * (180.0/M_PI);

            // use alternate representation if the yaw is smaller
            if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
            }
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "                      [ypr] :: " + to_string_with_precision(yaw, 4) + " "
                                            + to_string_with_precision(pitch, 4) + " "
                                            + to_string_with_precision(roll, 4)
                << "|" << std::endl;

            std::cout << "|                                                                   |" << std::endl;

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Deskewed points: " + std::to_string(this->deskew_size) << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Integrated states: " + std::to_string(this->propagated_size) << "|" << std::endl;
            std::cout << "|                                                                   |" << std::endl;

            std::cout << std::right << std::setprecision(2) << std::fixed;
            std::cout << "| Computation Time :: "
                << std::setfill(' ') << std::setw(6) << this->cpu_times.front()*1000. << " ms    // Avg: "
                << std::setw(6) << avg_comp_time*1000. << " / Max: "
                << std::setw(6) << *std::max_element(this->cpu_times.begin(), this->cpu_times.end())*1000.
                << "     |" << std::endl;
            std::cout << "| Cores Utilized   :: "
                << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
                << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
                << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
                                * this->numProcessors
                << "     |" << std::endl;
            std::cout << "| CPU Load         :: "
                << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
                << std::setw(6) << avg_cpu_usage << " / Max: "
                << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
                << "     |" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
                << "|" << std::endl;

            std::cout << "+-------------------------------------------------------------------+" << std::endl;

            }//end if(config.verbose)

            // Overwrite CPU stats
            this->mtx_cpu_stats.lock();
            this->cpu_time = cpu_times.front()*1000.0f;
            this->cpu_mean_time = avg_comp_time*1000.0f;
            this->cpu_max_time = *std::max_element(this->cpu_times.begin(), this->cpu_times.end())*1000.0f;
            this->cpu_cores = (cpu_percent/100.0f) * this->numProcessors;
            this->cpu_load = cpu_percent;
            this->cpu_max_load = *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end());
            this->ram_usage = resident_set/1000.0f;
            this->mtx_cpu_stats.unlock();

        }
