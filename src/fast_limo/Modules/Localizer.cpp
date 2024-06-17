#include "fast_limo/Modules/Localizer.hpp"

// class fast_limo::Localizer
    // public

        Localizer::Localizer() : scan_stamp(0.0), prev_scan_stamp(0.0), scan_dt(0.1), deskew_size(0), numProcessors(0),
                                imu_stamp(0.0), prev_imu_stamp(0.0), imu_dt(0.005), first_imu_stamp(0.0),
                                last_propagate_time_(-1.0), imu_calib_time_(3.0), gravity_(9.81), imu_calibrated_(false)
                            { 

            this->original_scan  = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->deskewed_scan  = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->pc2match       = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->final_raw_scan = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
            this->final_scan     = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
        }

        void Localizer::init(Config& cfg, bool one_thread){

            // To DO:
            //      - config better to be a shared obj between all fast_limo

            // Save config
            this->config = cfg;

            // Set One Threaded flag
            this->one_thread_ = one_thread;
            
            // Set num of threads
            this->num_threads_ = omp_get_max_threads();
            if(num_threads_ > config.num_threads) this->num_threads_ = config.num_threads;

            // Update Mapper config
            fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
            map.set_num_threads(this->num_threads_);
            map.set_match_points(config.ikfom.NUM_MATCH_POINTS);

            // Initialize Iterated Kalman Filter on Manifolds
            this->init_iKFoM();

            // Set buffer capacity
            this->imu_buffer.set_capacity(2000);

            // PCL filters setup
            this->crop_filter.setNegative(true);
            this->crop_filter.setMin(Eigen::Vector4f(config.filters.cropBoxMin[0], config.filters.cropBoxMin[1], config.filters.cropBoxMin[2], 1.0));
            this->crop_filter.setMax(Eigen::Vector4f(config.filters.cropBoxMax[0], config.filters.cropBoxMax[1], config.filters.cropBoxMax[2], 1.0));

            this->voxel_filter.setLeafSize(config.filters.leafSize[0], config.filters.leafSize[0], config.filters.leafSize[0]);

            // LiDAR sensor type
            this->set_sensor_type(config.sensor_type); 

            // IMU attitude
            this->imu_accel_sm_ = Eigen::Map<Eigen::Matrix3f>(config.intrinsics.imu_sm.data(), 3, 3);

            // Extrinsics
            this->extr.imu2baselink.t = - Eigen::Map<Eigen::Vector3f>(config.extrinsics.baselink2imu_t.data(), 3);
            Eigen::Matrix3f baselink2imu_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.baselink2imu_R.data(), 3, 3);
            this->extr.imu2baselink.R = baselink2imu_R.transpose();

            this->extr.imu2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.imu2baselink_T.block(0, 3, 3, 1) = this->extr.imu2baselink.t;
            this->extr.imu2baselink_T.block(0, 0, 3, 3) = this->extr.imu2baselink.R;

            /* imu_front --> base_link
            - Translation: [-0.870, -0.440, -0.170]
            - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
                        in RPY (radian) [0.000, -0.000, 0.000]
                        in RPY (degree) [0.000, -0.000, 0.000]
           */

            this->extr.lidar2baselink.t = - Eigen::Map<Eigen::Vector3f>(config.extrinsics.baselink2lidar_t.data(), 3);
            Eigen::Matrix3f baselink2lidar_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.baselink2lidar_R.data(), 3, 3);
            this->extr.lidar2baselink.R = baselink2lidar_R.transpose();

            this->extr.lidar2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.lidar2baselink_T.block(0, 3, 3, 1) = this->extr.lidar2baselink.t;
            this->extr.lidar2baselink_T.block(0, 0, 3, 3) = this->extr.lidar2baselink.R;

            /* pandar_front --> base_link
            - Translation: [-0.395, 0.885, -0.201]
            - Rotation: in Quaternion [-0.016, -0.006, -0.336, 0.942]
                        in RPY (radian) [-0.027, -0.022, -0.685]
                        in RPY (degree) [-1.553, -1.269, -39.253]
            */

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

            if(config.verbose){
                // set up buffer capacities
                this->imu_rates.set_capacity(1000);
                this->lidar_rates.set_capacity(1000);
                this->cpu_times.set_capacity(1000);
                this->cpu_percents.set_capacity(1000);
            }
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

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_pc2match_pointcloud(){
            return this->pc2match;
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

        State Localizer::get_state(){
            State out = this->state;
            out.p += this->state.pLI;                                                // position in body/base_link frame
            out.q = this->state.q * this->state.qLI;                                 // attitude in body/base_link frame
            out.v = this->state.q.toRotationMatrix().transpose() * this->state.v;    // local velocity vector
            return out;
        }

        double Localizer::get_propagate_time(){
            return this->last_propagate_time_;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////           Principal callbacks/threads        /////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp){

            auto start_time = chrono::system_clock::now();

            if(raw_pc->points.size() < 1){
                std::cout << "FAST_LIMO::Raw PointCloud is empty!\n";
                return;
            }

            if(!this->imu_calibrated_){
                // std::cout << "FAST_LIMO::IMU calibrating...!\n";
                return;
            }

            if(this->imu_buffer.empty()){
                std::cout << "FAST_LIMO::IMU buffer is empty!\n";
                return;
            }

            // std::cout << "FAST_LIMO::updatePointCloud()\n";

            // Remove NaNs
            std::vector<int> idx;
            raw_pc->is_dense = false;
            pcl::removeNaNFromPointCloud(*raw_pc, *raw_pc, idx);

            // Crop Box Filter (1 m^2)
            if(this->config.filters.crop_active){
                this->crop_filter.setInputCloud(raw_pc);
                this->crop_filter.filter(*raw_pc);
            }

            // std::cout << "crop filter done\n";

            // Distance & Time Rate filters
            static float min_dist = static_cast<float>(this->config.filters.min_dist);
            static int rate_value = this->config.filters.rate_value;
            std::function<bool(boost::range::index_value<PointType&, long>)> filter_f;
            
            if(this->config.filters.dist_active && this->config.filters.rate_active){
                filter_f = [&min_dist, &rate_value](boost::range::index_value<PointType&, long> p)
                    { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist)
                                && (p.index()%rate_value == 0); };
            }
            else if(this->config.filters.dist_active){
                filter_f = [&min_dist](boost::range::index_value<PointType&, long> p)
                    { return Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist; };
            }
            else if(this->config.filters.rate_active){
                filter_f = [&rate_value](boost::range::index_value<PointType&, long> p)
                    { return p.index()%rate_value == 0; };
            }else{
                filter_f = [](boost::range::index_value<PointType&, long> p)
                    { return true; };
            }
            auto filtered_pc = raw_pc->points 
                        | boost::adaptors::indexed()
                        | boost::adaptors::filtered(filter_f);

            pcl::PointCloud<PointType>::Ptr input_pc (boost::make_shared<pcl::PointCloud<PointType>>());
            for (auto it = filtered_pc.begin(); it != filtered_pc.end(); it++) {
                input_pc->points.push_back(it->value());
            }
             
            if(this->config.debug) // debug only
                this->original_scan = boost::make_shared<pcl::PointCloud<PointType>>(*input_pc); // LiDAR frame

            // Motion compensation
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_pc_ = this->deskewPointCloud(input_pc, time_stamp);
            /*NOTE: deskewed_Xt2_pc_ should be in base_link frame w.r.t last predicted state (Xt2) */

            // std::cout << "Pointcloud deskewed\n";

            if(deskewed_Xt2_pc_->points.size() > 1){

                // Voxel Grid Filter
                if (this->config.filters.voxel_active) { 
                    pcl::PointCloud<PointType>::Ptr current_scan_
                        (boost::make_shared<pcl::PointCloud<PointType>>(*deskewed_Xt2_pc_));
                    this->voxel_filter.setInputCloud(current_scan_);
                    this->voxel_filter.filter(*current_scan_);
                    this->pc2match = current_scan_;
                } else {
                    this->pc2match = deskewed_Xt2_pc_;
                }

                // std::cout << "voxel grid filter applied\n";

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

                // std::cout << "IKFOM measurment updated\n";

                    // Update current state estimate
                corrected_state.b.gyro  = this->state.b.gyro;
                corrected_state.b.accel = this->state.b.accel;
                this->state    = corrected_state;
                this->state.w  = this->last_imu.ang_vel;

                this->mtx_ikfom.unlock();

                // Get estimated offset
                if(config.ikfom.estimate_extrinsics)
                    this->extr.lidar2baselink_T = this->state.get_extr_RT();

                // Transform deskewed pc 
                    // Get deskewed scan to add to map
                pcl::PointCloud<PointType>::Ptr mapped_scan (boost::make_shared<pcl::PointCloud<PointType>>());
                pcl::transformPointCloud (*this->pc2match, *mapped_scan, this->state.get_RT());
                /*NOTE: pc2match must be in base_link frame w.r.t Xt2 frame for this transform to work.
                        mapped_scan is in world/global frame.
                */

                /*To DO:
                    - mapped_scan --> segmentation of dynamic objects
                */

                    // Get final scan to output (in world/global frame)
                pcl::transformPointCloud (*this->pc2match, *this->final_scan, this->state.get_RT()); // mapped_scan = final_scan (for now)

                if(this->config.debug) // save final scan without voxel grid
                    pcl::transformPointCloud (*deskewed_Xt2_pc_, *this->final_raw_scan, this->state.get_RT());

                // std::cout << "final scan!\n";

                // Add scan to map
                fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
                map.add(mapped_scan, this->scan_stamp);

            }

            auto end_time = chrono::system_clock::now();
            elapsed_time = end_time - start_time;

            if(this->config.verbose){
                // fill stats
                if(this->prev_scan_stamp > 0.0) this->lidar_rates.push_front( 1. / (this->scan_stamp - this->prev_scan_stamp) );
                if(calibrating > 0) this->cpu_times.push_front(elapsed_time.count());
                else this->cpu_times.push_front(0.0);
                
                if(calibrating < UCHAR_MAX) calibrating++;

                // debug thread
                this->debug_thread = std::thread( &Localizer::debugVerbose, this );
                this->debug_thread.detach();
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
                        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... \n";
                        std::cout.flush();
                        print = false;
                    }

                } else {

                    std::cout << "done!" << std::endl << std::endl;

                    gyro_avg /= num_samples;
                    accel_avg /= num_samples;

                    Eigen::Vector3f grav_vec (0., 0., this->gravity_);

                    this->state.q = imu.q;

                    if (this->config.gravity_align) {

                        std::cout << " Acceleration average: \n";
                        std::cout << accel_avg << std::endl;

                        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
                        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));
                        
                        std::cout << " Gravity average: \n";
                        std::cout << grav_vec << std::endl;

                        // set gravity aligned orientation
                        this->state.q = grav_q;

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

                    this->imu_calibrated_ = true;

                    // Set initial KF state
                    this->init_iKFoM_state();

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

                // std::cout << "Receiving IMU meas\n";

                if(this->config.verbose) this->imu_rates.push_front( 1./imu.dt );

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
                if(not this->one_thread_)
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
                Eigen::Vector4f p4_lidar = S.get_extr_RT_inv() /* baselink2lidar */ * S.get_RT_inv() * match.get_point();
                Eigen::Vector4f p4_imu   = S.get_extr_RT() /* lidar2baselink */ * p4_lidar;
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
                if (config.ikfom.estimate_extrinsics) H.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

                // Measurement: distance to the closest plane
                h(i) = -match.dist;
            }
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

            double dt = imu.dt;
            this->mtx_ikfom.lock();
            this->_iKFoM.predict(dt, Q, in);
            this->mtx_ikfom.unlock();

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

            input_ikfom input;
            double dt;
            for (; imu_it != end_imu_it; imu_it++) {
                const IMUmeas& imu = *imu_it;

                input.acc  = imu.lin_accel.cast<double>();
                input.gyro = imu.ang_vel.cast<double>();
                dt = imu.dt;

                this->mtx_ikfom.lock();
                this->_iKFoM.predict(dt, Q, input);
                this->mtx_ikfom.unlock();
            }

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
                // std::vector<double> (23, 0.001) /*config.ikfom.LIMITS*/
            );
        }

        void Localizer::init_iKFoM_state() {
            state_ikfom init_state = this->_iKFoM.get_x();
            init_state.rot = this->state.q.cast<double> ();
            init_state.grav = /*MTK::*/S2(Eigen::Vector3d(0., 0., -this->gravity_));
            init_state.bg = this->state.b.gyro.cast<double>();
            init_state.ba = this->state.b.accel.cast<double>();

            init_state.offset_R_L_I = /*MTK::*/SO3(this->extr.lidar2baselink.R.cast<double>());
            init_state.offset_T_L_I = this->extr.lidar2baselink.t.cast<double>();
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
            } else {
                std::cout << "-------------------------------------------------------------------\n";
                std::cout << "FAST_LIMO::FATAL ERROR: LiDAR sensor type unknown or not specified!\n";
                std::cout << "-------------------------------------------------------------------\n";
                return boost::make_shared<pcl::PointCloud<PointType>>();
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
            if (config.time_offset) {
                offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
            }

            // std::cout << "timestamps:\n";

            // build list of unique timestamps and indices of first point with each timestamp
            for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
                timestamps.push_back(extract_point_time(*it) + offset);
                unique_time_indices.push_back(it->index());

                // std::cout << std::setprecision(20) << extract_point_time(*it) + offset << std::endl;
            }
            unique_time_indices.push_back(deskewed_scan_->points.size());

            // std::cout << "timestamps size: " << timestamps.size() << std::endl;

            // To DO: check which option from above works better
            // int median_pt_index = timestamps.size() / 2;
            // this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point
            this->scan_stamp = start_time;

            // IMU prior & deskewing for second scan onwards
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
            frames = this->integrateImu(this->prev_scan_stamp, this->state.q, this->state.p,
                                        this->state.v, timestamps);
            this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

            // std::cout << "Imu integrated: " << frames.size() << std::endl;

            // TO DO: if there are no frames between the start and end of the sweep
            // that probably means that there's a sync issue
            if (frames.size() < timestamps.size()) {
                std::cout << "FAST_LIMO::FATAL ERROR: Bad time sync between LiDAR and IMU!\n";
                std::cout << "frames.size(): " << frames.size() << std::endl;
                std::cout << "timestamps.size(): " << timestamps.size() << std::endl;
                return boost::make_shared<pcl::PointCloud<PointType>>();
            }

            if(frames.size() < 1) return boost::make_shared<pcl::PointCloud<PointType>>();

            // deskewed pointcloud w.r.t last known state prediction
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_scan_ (boost::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_scan_->points.resize(deskewed_scan_->points.size());

            this->last_state = fast_limo::State(frames[frames.size()-1]);
            // std::cout << "LAST STATE: " << this->last_state.p << std::endl;

            // for(int i=0; i < frames.size(); i++) std::cout << "FRAME: " << fast_limo::State(frames[i]).p << std::endl;

            #pragma omp parallel for num_threads(this->num_threads_)
            for (int i = 0; i < timestamps.size(); i++) {

                Eigen::Matrix4f T = frames[i] * this->extr.lidar2baselink_T;

                // transform point to world frame
                for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {

                    // world frame deskewed pc
                    auto &pt = deskewed_scan_->points[k];
                    pt.getVector4fMap()[3] = 1.;
                    pt.getVector4fMap() = T * pt.getVector4fMap(); // world/global frame

                    // Xt2 frame deskewed pc
                    auto &pt2 = deskewed_Xt2_scan_->points[k];
                    pt2.getVector4fMap() = this->last_state.get_RT_inv() * pt.getVector4fMap(); // Xt2 frame
                    pt2.intensity = pt.intensity;
                }
            }

            if(this->config.debug) // debug only
                this->deskewed_scan = deskewed_scan_;

            return deskewed_Xt2_scan_; 
        }

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        Localizer::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                                Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps){

            const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

            if(sorted_timestamps.size() < 1){
                std::cout << "FAST_LIMO::integrateImu() invalid input: sorted timestamps are empty!\n";
                return empty;
            }

            if(start_time > sorted_timestamps.front()){
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
                if(not this->one_thread_){ // Wait for the latest IMU data
                    std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
                    this->cv_imu_stamp.wait(lock, [this, &end_time]{ return this->imu_buffer.front().stamp >= end_time; });
                }else
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
            double avg_imu_rate =
                std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size();
            double avg_lidar_rate =
                std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size();

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

            State final_state = this->get_state();

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

        }