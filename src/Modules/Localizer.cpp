#include "fast_limo/Modules/Localizer.hpp"

// class fast_limo::Localizer
    // public

        Localizer::Localizer() : scan_stamp(0.0), prev_scan_stamp(0.0), scan_dt(0.1), 
                                imu_stamp(0.0), prev_imu_stamp(0.0), imu_dt(0.005), first_imu_stamp(0.0),
                                imu_calib_time_(3.0), gravity_(9.81), imu_calibrated_(false), gravity_align_(true),
                                calibrate_accel_(true), calibrate_gyro_(true), debug_(false), voxel_flag_(true) { 

            this->original_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->deskewed_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->pc2match      = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->final_scan    = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
        }

        void Localizer::init(double t){
            
            // Set num of threads
            this->num_threads_ = omp_get_max_threads();

            // Initialize Iterated Kalman Filter on Manifolds
            this->init_iKFoM();

            // Set buffer capacity
            this->imu_buffer.set_capacity(2000);

            // PCL filters setup
            this->crop_filter.setNegative(true);
            this->crop_filter.setMin(Eigen::Vector4f(-1.0, -1.0, -1.0, 1.0));
            this->crop_filter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));

            this->voxel_filter.setLeafSize(0.25, 0.25, 0.25);

            // LiDAR sensor type
            this->sensor = fast_limo::SensorType::HESAI;

            // IMU attitude
            this->imu_accel_sm_ = Eigen::Matrix3f::Identity();

            // Extrinsics
                // To DO: fill with parameters
            this->extr.baselink2imu.t = Eigen::Vector3f(0., 0., 0.);
            this->extr.baselink2imu.R = Eigen::Matrix3f::Identity();

            this->extr.baselink2imu_T = Eigen::Matrix4f::Identity();
            this->extr.baselink2imu_T.block(0, 3, 3, 1) = this->extr.baselink2imu.t;
            this->extr.baselink2imu_T.block(0, 0, 3, 3) = this->extr.baselink2imu.R;

                // To DO: fill with parameters
            this->extr.baselink2lidar.t = Eigen::Vector3f(0., -0.8639, 0.);
            this->extr.baselink2lidar.R = Eigen::Matrix3f::Identity();

            this->extr.baselink2lidar_T = Eigen::Matrix4f::Identity();
            this->extr.baselink2lidar_T.block(0, 3, 3, 1) = this->extr.baselink2lidar.t;
            this->extr.baselink2lidar_T.block(0, 0, 3, 3) = this->extr.baselink2lidar.R;

            // Debugging
            this->debug_ = true;

        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_pointcloud(){
            return this->final_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_orig_pointcloud(){
            return this->original_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_deskewed_pointcloud(){
            return this->deskewed_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_pc2match_pointcloud(){
            return this->pc2match;
        }

        State& Localizer::get_state(){
            this->state.w = this->last_imu.ang_vel;
            return this->state;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////           Principal callbacks/threads        /////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp){

            // TO DO: check IMU callibration is finished
            if (!this->imu_calibrated_ || this->imu_buffer.empty() ){
                std::cout << "FAST_LIMO::IMU buffer is empty!\n";
                return;
            }

            std::cout << "FAST_LIMO::updatePointCloud()\n";

            // Crop Box Filter (1 m^2)
            this->crop_filter.setInputCloud(raw_pc);
            this->crop_filter.filter(*raw_pc);

            std::cout << "crop filter done\n";

            if(this->debug_) // debug only
                this->original_scan = boost::make_shared<pcl::PointCloud<PointType>>(*raw_pc); // base_link/body frame

            // Motion compensation
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_pc_ = this->deskewPointCloud(raw_pc, time_stamp);
            /*NOTE: deskewed_Xt2_pc_ should be in last predicted state (Xt2) frame*/

            std::cout << "Pointcloud deskewed\n";

            if(deskewed_Xt2_pc_->points.size() > 1){

                // Voxel Grid Filter
                if (this->voxel_flag_) { 
                    pcl::PointCloud<PointType>::Ptr current_scan_
                        (boost::make_shared<pcl::PointCloud<PointType>>(*deskewed_Xt2_pc_));
                    this->voxel_filter.setInputCloud(current_scan_);
                    this->voxel_filter.filter(*current_scan_);
                    this->pc2match = current_scan_;
                } else {
                    this->pc2match = deskewed_Xt2_pc_;
                }

                std::cout << "voxel grid filter applied\n";

                // iKFoM observation stage
                this->mtx_ikfom.lock();

                    // Update iKFoM measurements (after prediction)
                double solve_time = 0.0;
                this->_iKFoM.update_iterated_dyn_share_modified(0.001 /*LiDAR noise*/, 5.0/*Degeneracy threshold*/, 
                                                                solve_time/*solving time elapsed*/, false/*print degeneracy values flag*/);

                    /*NOTE: update_iterated_dyn_share_modified() will trigger the matching procedure ( see "use-ikfom.cpp" )
                    in order to update the measurement stage of the KF with the computed point-to-plane distances*/

                    // Get output state from iKFoM
                fast_limo::State corrected_state = fast_limo::State(this->_iKFoM.get_x()); 

                std::cout << "IKFOM measurment updated\n";

                    // Update current state estimate
                corrected_state.b.gyro  = this->state.b.gyro;
                corrected_state.b.accel = this->state.b.accel;
                this->state = corrected_state;

                this->mtx_ikfom.unlock();

                // Transform deskewed pc 
                pcl::transformPointCloud (*this->pc2match, *this->final_scan, this->state.get_RT());
                // pc2match must be in Xt2 frame for this transform to work properly

                std::cout << "final scan!\n";

                // Add scan to map
                fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
                map.add(final_scan, this->scan_stamp);

            }

            this->prev_scan_stamp = this->scan_stamp;

        }

        void Localizer::updateIMU(IMUmeas& raw_imu){

            this->imu_stamp = raw_imu.stamp;
            IMUmeas imu = this->imu2baselink(raw_imu);

            if(this->first_imu_stamp == 0.0) this->first_imu_stamp = imu.stamp;

            // IMU calibration procedure - do for three seconds
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
                        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... ";
                        std::cout.flush();
                        print = false;
                    }

                } else {

                    std::cout << "done" << std::endl << std::endl;

                    gyro_avg /= num_samples;
                    accel_avg /= num_samples;

                    Eigen::Vector3f grav_vec (0., 0., this->gravity_);

                    if (this->gravity_align_) {

                        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
                        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));

                        // set gravity aligned orientation
                        this->state.q = grav_q;
                        // this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();

                        // rpy
                        auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
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

                    if (this->calibrate_accel_) {

                        // subtract gravity from avg accel to get bias
                        this->state.b.accel = accel_avg - grav_vec;

                        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
                    }

                    if (this->calibrate_gyro_) {

                        this->state.b.gyro = gyro_avg;

                        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
                    }

                    this->imu_calibrated_ = true;

                    // Set initial KF state
                    this->init_iKFoM_state();

                }

            } else {

                // std::cout << "Receiving IMU meas\n";

                // this->imu_rates.push_back( 1./imu.dt );

                // Apply the calibrated bias to the new IMU measurements
                Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * imu.lin_accel) - this->state.b.accel;
                Eigen::Vector3f ang_vel_corrected = imu.ang_vel - this->state.b.gyro;

                imu.lin_accel = lin_accel_corrected;
                imu.ang_vel   = ang_vel_corrected;

                // Store calibrated IMU measurements into imu buffer for manual integration later.
                this->mtx_imu.lock();
                this->imu_buffer.push_front(imu);
                this->mtx_imu.unlock();

                // std::cout << "IMU buffer filled: " << imu_buffer.size() << std::endl;

                // Notify callbackPointCloud::imuMeasFromTimeRange() thread that IMU data exists for this time
                this->cv_imu_stamp.notify_one();

                // std::cout << "PC thread notified\n";

                // iKFoM propagate state
                this->propagateImu(imu);

                // std::cout << "IKFOM propagate IMU\n";

            }

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          KF measurement model        /////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::calculate_H(const state_ikfom& s, const Matches& matches, Eigen::MatrixXd& H, Eigen::VectorXd& h){
            int Nmatches = matches.size();
            H = Eigen::MatrixXd::Zero(Nmatches, 12);
            h.resize(Nmatches);
            State S(s);

            // For each match, calculate its derivative and distance
            for (int i = 0; i < matches.size(); ++i) {
                Match match = matches[i];
                Eigen::Vector4f p4_lidar = extr.baselink2lidar_T.inverse()/* lidar2baselink_T */ * S.get_RT_inv() * match.get_point();
                Eigen::Vector4f p4_imu   = extr.baselink2lidar_T * p4_lidar;
                Eigen::Vector4f normal   = match.plane.get_normal();

                // Rotation matrices
                Eigen::Matrix3f R_inv = s.rot.conjugate().toRotationMatrix().cast<float>();
                Eigen::Matrix3f I_R_L_inv = s.offset_R_L_I.conjugate().toRotationMatrix().cast<float>();

                // Set correct dimensions
                Eigen::Vector3f p_lidar, p_imu, n;
                p_lidar = p4_lidar.head(3);
                p_imu   = p4_imu.head(3);
                n       = normal.head(3);

                if( (fabs(p4_lidar(3)-1.0) > 0.01) || (fabs(p4_imu(3)-1.0) > 0.01) ) std::cout << "FAST_LIMO::Localizer::calculate_H() "
                                                                                                << "global to local transform with low precision!\n";

                // Calculate H (:= dh/dx)
                Eigen::Vector3f C = R_inv * n;
                Eigen::Vector3f B = p_lidar.cross(I_R_L_inv * C);
                Eigen::Vector3f A = p_imu.cross(C);
                
                H.block<1, 6>(i,0) << n(0), n(1), n(2), A(0), A(1), A(2);
                if (false/*Config.estimate_extrinsics*/) H.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

                // Measurement: distance to the closest plane
                h(i) = -match.dist;
            }
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
                3 /*Config.MAX_NUM_ITERS*/,
                std::vector<double> (23, 0.001) /*Config.LIMITS*/
            );
        }

        void Localizer::init_iKFoM_state() {
            state_ikfom init_state = this->_iKFoM.get_x();
            init_state.rot = this->state.q.cast<double> ();
            init_state.grav = /*MTK::*/S2(Eigen::Vector3d(0., 0., -this->gravity_));
            init_state.bg = this->state.b.gyro.cast<double>();
            init_state.ba = this->state.b.accel.cast<double>();

            init_state.offset_R_L_I = /*MTK::*/SO3(this->extr.baselink2lidar.R.cast<double>());
            init_state.offset_T_L_I = this->extr.baselink2lidar.t.cast<double>();
            this->_iKFoM.change_x(init_state);

            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = this->_iKFoM.get_P();
            init_P.setIdentity();
            init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
            init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
            init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
            init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
            init_P(21,21) = init_P(22,22) = 0.00001; 
            
            this->_iKFoM.change_P(init_P);
        }

        IMUmeas Localizer::imu2baselink(IMUmeas& imu){

            IMUmeas imu_baselink;

            double dt = imu.stamp - this->prev_imu_stamp;
            
            if ( (dt == 0.) || (dt > 0.1) ) { dt = 1.0/200.0; }

            // Transform angular velocity (will be the same on a rigid body, so just rotate to ROS convention)
            Eigen::Vector3f ang_vel_cg = this->extr.baselink2imu.R * imu.ang_vel;

            static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

            // Transform linear acceleration (need to account for component due to translational difference)
            Eigen::Vector3f lin_accel_cg = this->extr.baselink2imu.R * imu.lin_accel;

            lin_accel_cg = lin_accel_cg
                            + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extr.baselink2imu.t)
                            + ang_vel_cg.cross(ang_vel_cg.cross(-this->extr.baselink2imu.t));

            ang_vel_cg_prev = ang_vel_cg;

            imu_baselink.ang_vel   = ang_vel_cg;
            imu_baselink.lin_accel = lin_accel_cg;
            imu_baselink.dt        = dt;
            imu_baselink.stamp     = imu.stamp;

            return imu_baselink;

        }

        void Localizer::propagateImu(const IMUmeas& imu){
            input_ikfom in;
            in.acc = imu.lin_accel.cast<double>();
            in.gyro = imu.ang_vel.cast<double>();

            Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
            // TO DO: get config params from covariance
            Q.block<3, 3>(0, 0) = 0.0001 /*Config.cov_gyro*/ * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(3, 3) = 0.01 /*Config.cov_acc*/ * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(6, 6) = 0.00001 /*Config.cov_bias_gyro*/ * Eigen::Matrix<double, 3, 3>::Identity();
            Q.block<3, 3>(9, 9) = 0.0001 /*Config.cov_bias_acc*/ * Eigen::Matrix<double, 3, 3>::Identity();

            double dt = imu.dt;
            this->mtx_ikfom.lock();
            this->_iKFoM.predict(dt, Q, in);
            this->mtx_ikfom.unlock();
        }

        pcl::PointCloud<PointType>::Ptr
        Localizer::deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time){

            pcl::PointCloud<PointType>::Ptr deskewed_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
            deskewed_scan_->points.resize(pc->points.size());

            // individual point timestamps should be relative to this time
            double sweep_ref_time = start_time;

            // sort points by timestamp and build list of timestamps
            std::function<bool(const PointType&, const PointType&)> point_time_cmp;
            std::function<bool(boost::range::index_value<PointType&, long>,
                                boost::range::index_value<PointType&, long>)> point_time_neq;
            std::function<double(boost::range::index_value<PointType&, long>)> extract_point_time;

            if (this->sensor == fast_limo::SensorType::OUSTER) {

                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.t < p2.t; };
                point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                                    boost::range::index_value<PointType&, long> p2)
                { return p1.value().t != p2.value().t; };
                extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
                { return sweep_ref_time + pt.value().t * 1e-9f; };

            } else if (this->sensor == fast_limo::SensorType::VELODYNE) {

                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.time < p2.time; };
                point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                                    boost::range::index_value<PointType&, long> p2)
                { return p1.value().time != p2.value().time; };
                extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
                { return sweep_ref_time + pt.value().time; };

            } else if (this->sensor == fast_limo::SensorType::HESAI) {

                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                                    boost::range::index_value<PointType&, long> p2)
                { return p1.value().timestamp != p2.value().timestamp; };
                extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
                { return pt.value().timestamp; };

            } else if (this->sensor == fast_limo::SensorType::LIVOX) {
                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                                    boost::range::index_value<PointType&, long> p2)
                { return p1.value().timestamp != p2.value().timestamp; };
                extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
                { return pt.value().timestamp * 1e-9f; };
            }

            // copy points into deskewed_scan_ in order of timestamp
            std::partial_sort_copy(pc->points.begin(), pc->points.end(),
                                    deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

            // filter unique timestamps
            auto points_unique_timestamps = deskewed_scan_->points
                                            | boost::adaptors::indexed()
                                            | boost::adaptors::adjacent_filtered(point_time_neq);

            // extract timestamps from points and put them in their own list
            std::vector<double> timestamps;
            std::vector<int> unique_time_indices;

            // compute offset between sweep reference time and first point timestamp
            double offset = 0.0;
            // if (this->time_offset_) {
                offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
            // }

            // std::cout << "timestamps:\n";

            // build list of unique timestamps and indices of first point with each timestamp
            for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
                timestamps.push_back(extract_point_time(*it) + offset);
                unique_time_indices.push_back(it->index());

                // std::cout << std::setprecision(20) << extract_point_time(*it) + offset << std::endl;
            }
            unique_time_indices.push_back(deskewed_scan_->points.size());

            int median_pt_index = timestamps.size() / 2;
            this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point

            // IMU prior & deskewing for second scan onwards
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
            frames = this->integrateImu(this->prev_scan_stamp, this->state.q, this->state.p,
                                        this->state.v, timestamps);
            // this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

            // TO DO: if there are no frames between the start and end of the sweep
            // that probably means that there's a sync issue
            if (frames.size() != timestamps.size()) {
                std::cout << "FAST_LIMO::FATAL ERROR: Bad time sync between LiDAR and IMU!\n";
                std::cout << "frames.size(): " << frames.size() << std::endl;
                std::cout << "timestamps.size(): " << timestamps.size() << std::endl;
                // return boost::make_shared<pcl::PointCloud<PointType>>();
            }

            if(frames.size() < 1) return boost::make_shared<pcl::PointCloud<PointType>>();

            // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
            // this->T_prior = frames[median_pt_index];

            // deskewed pointcloud w.r.t last known state prediction
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_scan_->points.resize(deskewed_scan_->points.size());

            this->last_state = fast_limo::State(frames[frames.size()-1]);
            std::cout << "LAST STATE: " << this->last_state.p << std::endl;

            for(int i=0; i < frames.size(); i++) std::cout << "FRAME: " << fast_limo::State(frames[i]).p << std::endl;

            #pragma omp parallel for num_threads(this->num_threads_)
            for (int i = 0; i < timestamps.size(); i++) {

                Eigen::Matrix4f T = frames[i] * this->extr.baselink2lidar_T;

                // transform point to world frame
                // TO DO: deskewed scan must be in lidar frame --> taking into account last frame (maybe pick frames from iKFoM)
                for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {

                    // world frame deskewed pc
                    auto &pt = deskewed_scan_->points[k];
                    pt.getVector4fMap()[3] = 1.;
                    pt.getVector4fMap() = T * pt.getVector4fMap();

                    // Xt2 frame deskewed pc
                    auto &pt2 = deskewed_Xt2_scan_->points[k];
                    // pt2.getVector4fMap()[3] = 1.;
                    pt2.getVector4fMap() = this->last_state.get_RT_inv() * fast_limo::State(this->extr.baselink2lidar_T).get_RT_inv() * pt.getVector4fMap();
                }
            }

            if(this->debug_) // debug only
                this->deskewed_scan = deskewed_scan_;

            // this->deskew_status = true;

            return deskewed_Xt2_scan_; 
        }

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        Localizer::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                                Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps){

            const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

            if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
                // invalid input, return empty vector
                std::cout << "FAST_LIMO::integrateImu() invalid input: sorted timestamps are not consistent\n";
                return empty;
            }

            boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it;
            boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it;
            if (not this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it)) {
                // not enough IMU measurements, return empty vector
                std::cout << "FAST_LIMO::imuMeasFromTimeRange(): not enough IMU measurements\n";
                return empty;
            }

            // Backwards integration to find pose at first IMU sample
            const IMUmeas& f1 = *begin_imu_it;
            const IMUmeas& f2 = *(begin_imu_it+1);

            // Save last IMU being used
            this->last_imu = *end_imu_it;

            // Time between first two IMU samples
            double dt = f2.dt;

            // Time between first IMU sample and start_time
            double idt = start_time - f1.stamp;

            // Angular acceleration between first two IMU samples
            Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
            Eigen::Vector3f alpha = alpha_dt / dt;

            // Average angular velocity (reversed) between first IMU sample and start_time
            Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);

            // Set q_init to orientation at first IMU sample
            q_init = Eigen::Quaternionf (
                q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
                q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
                q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
                q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
            );
            q_init.normalize();

            // Average angular velocity between first two IMU samples
            Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;

            // Orientation at second IMU sample
            Eigen::Quaternionf q2 (
                q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
                q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
                q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
                q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
            );
            q2.normalize();

            // Acceleration at first IMU sample
            Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
            a1[2] -= this->gravity_;

            // Acceleration at second IMU sample
            Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
            a2[2] -= this->gravity_;

            // Jerk between first two IMU samples
            Eigen::Vector3f j = (a2 - a1) / dt;

            // Set v_init to velocity at first IMU sample (go backwards from start_time)
            v_init -= a1*idt + 0.5*j*idt*idt;

            // Set p_init to position at first IMU sample (go backwards from start_time)
            p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

            return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);

        }

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        Localizer::integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                        const std::vector<double>& sorted_timestamps,
                                        boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it,
                                        boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it){

            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

            // Initialization
            Eigen::Quaternionf q = q_init;
            Eigen::Vector3f p = p_init;
            Eigen::Vector3f v = v_init;
            Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
            a[2] -= this->gravity_;

            // Iterate over IMU measurements and timestamps
            auto prev_imu_it = begin_imu_it;
            auto imu_it = prev_imu_it + 1;

            auto stamp_it = sorted_timestamps.begin();

            for (; imu_it != end_imu_it; imu_it++) {

                const IMUmeas& f0 = *prev_imu_it;
                const IMUmeas& f = *imu_it;

                // Time between IMU samples
                double dt = f.dt;

                // Angular acceleration
                Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
                Eigen::Vector3f alpha = alpha_dt / dt;

                // Average angular velocity
                Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

                // Orientation
                q = Eigen::Quaternionf (
                q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
                q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
                q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
                q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
                );
                q.normalize();

                // Acceleration
                Eigen::Vector3f a0 = a;
                a = q._transformVector(f.lin_accel);
                a[2] -= this->gravity_;

                // Jerk
                Eigen::Vector3f j_dt = a - a0;
                Eigen::Vector3f j = j_dt / dt;

                // Interpolate for given timestamps
                while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
                // Time between previous IMU sample and given timestamp
                double idt = *stamp_it - f0.stamp;

                // Average angular velocity
                Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;

                // Orientation
                Eigen::Quaternionf q_i (
                    q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
                    q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
                    q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
                    q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
                );
                q_i.normalize();

                // Position
                Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

                // Transformation
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
                T.block(0, 3, 3, 1) = p_i;

                imu_se3.push_back(T);

                stamp_it++;
                }

                // Position
                p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

                // Velocity
                v += a0*dt + 0.5*j_dt*dt;

                prev_imu_it = imu_it;

            }

            return imu_se3;

        }

        bool Localizer::imuMeasFromTimeRange(double start_time, double end_time,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it) {

            if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
                // Wait for the latest IMU data
                std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
                this->cv_imu_stamp.wait(lock, [this, &end_time]{ return this->imu_buffer.front().stamp >= end_time; });
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