#include "fast_limo/Modules/Localizer.hpp"

Localizer::Localizer() : scan_stamp(0.0), prev_scan_stamp(0.0), scan_dt(0.1), 
                        imu_stamp(0.0), prev_imu_stamp(0.0), imu_dt(0.005), first_imu_stamp(0.0),
                        imu_calib_time_(3.0), gravity_(9.81), imu_calibrated_(false), gravity_align_(true),
                        calibrate_accel_(true), calibrate_gyro_(true), debug_(false), voxel_flag_(true) { 

    this->original_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->deskewed_scan = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->final_scan    = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
    this->pc2match      = pcl::PointCloud<PointType>::ConstPtr (boost::make_shared<const pcl::PointCloud<PointType>>());
}

void Localizer::init(double t){

    this->num_threads_ = omp_get_max_threads();

    this->crop_filter.setNegative(true);
    this->crop_filter.setMin(Eigen::Vector4f(-1.0, -1.0, -1.0, 1.0));
    this->crop_filter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));

    this->voxel_filter.setLeafSize(0.25, 0.25, 0.25);

    this->sensor = fast_limo::SensorType::HESAI;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////           Principal callbacks/threads        /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Localizer::updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double& time_stamp){

    // TO DO: check IMU callibration is finished

    // Crop Box Filter (1 m^2)
    this->crop_filter.setInputCloud(raw_pc);
    this->crop_filter.filter(*raw_pc);

    if(this->debug_) // debug only
        this->original_scan = boost::make_shared<pcl::PointCloud<PointType>>(*raw_pc); // should be in base_link/body frame

    // Motion compensation
    pcl::PointCloud<PointType>::Ptr deskewed_pc_ (boost::make_shared<pcl::PointCloud<PointType>>());
    deskewed_pc_ = this->deskewPointCloud(raw_pc, time_stamp);
    /*NOTE: deskewed_pc_ should be in global/world frame*/

    if(this->debug) // debug only
        this->deskewed_scan = deskewed_pc_;

    // Voxel Grid Filter
    if (this->voxel_flag_) { 
        pcl::PointCloud<PointType>::Ptr current_scan_
            (boost::make_shared<pcl::PointCloud<PointType>>(*deskewed_pc_));
        this->voxel_filter.setInputCloud(current_scan_);
        this->voxel_filter.filter(*current_scan_);
        this->pc2match = current_scan_;
    } else {
        this->pc2match = deskewed_pc_;
    }

    /*TO DO: pc2match should be in lidar frame --> will be transported into world by the IMU predicted final state before landmark detection*/

    // Update iKFoM measurements (after prediction)
    double solve_time = 0.0;
    this->_iKFoM.update_iterated_dyn_share_modified(0.001 /*LiDAR noise*/, 5.0/*Degeneracy threshold*/, 
                                                    solve_time/*solving time elapsed*/, false/*print degeneracy values flag*/);

    /*NOTE: update_iterated_dyn_share_modified() will trigger a matching procedure ( see "use-ikfom.cpp" )
    in order to update the measurement stage of the KF with point-to-plane distances as new measurements*/

    // Get output state from iKFoM
    fast_limo::State corrected_state = fast_limo::State(this->_iKFoM.get_x()); 

    // Transform deskewed pc 
    pcl::transformPointCloud (*this->pc2match, *this->final_scan, corrected_state.get_RT());
    // TO DO: pc2match must be in lidar frame for this transform to work properly

    /* To DO:
        - transform deskewed pc with output state
        - publish output pc
        - add output pc to map
        - save & publish output state
        - start thread for debugging info (print out)
    */

   this->prev_scan_stamp = this->scan_stamp;

}

void Localizer::updateIMU(IMUmeas& raw_imu){

    this->imu_stamp = raw_imu.stamp;
    IMUmeas imu = this->imu2baselink(raw_imu);

    // this->first_imu_received = true;

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
                this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
                // this->lidarPose.q = this->state.q;

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

        }

    } else {

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

        // Notify the callbackPointCloud::imuMeasFromTimeRange() thread that IMU data exists for this time
        this->cv_imu_stamp.notify_one();

        // iKFoM propagate state
        this->propagateImu(imu);

    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////          Aux. functions        ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IMUmeas& Localizer::imu2baselink(IMUmeas& imu){

    IMUmeas imu_baselink;

    double dt = imu.stamp - this->prev_imu_stamp;
    
    if (dt == 0.) { dt = 1.0/200.0; }

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
    // Q.block<3, 3>(0, 0) = Config.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();
    // Q.block<3, 3>(3, 3) = Config.cov_acc * Eigen::Matrix<double, 3, 3>::Identity();
    // Q.block<3, 3>(6, 6) = Config.cov_bias_gyro * Eigen::Matrix<double, 3, 3>::Identity();
    // Q.block<3, 3>(9, 9) = Config.cov_bias_acc * Eigen::Matrix<double, 3, 3>::Identity();

    double dt = imu.dt;
    this->_iKFoM.predict(dt, Q, in);
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

    // build list of unique timestamps and indices of first point with each timestamp
    for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
        timestamps.push_back(extract_point_time(*it) + offset);
        unique_time_indices.push_back(it->index());
    }
    unique_time_indices.push_back(deskewed_scan_->points.size());

    int median_pt_index = timestamps.size() / 2;
    this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point

    // TO DO: don't process scans until IMU data is present

    // if (!this->first_valid_scan) {
    //     if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
    //     return;
    //     }

    //     this->first_valid_scan = true;
    //     this->T_prior = this->T; // assume no motion for the first scan
    //     pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extr.baselink2lidar_T);
    //     this->deskewed_scan = deskewed_scan_;
    //     this->deskew_status = true;
    //     return;
    // }

    // IMU prior & deskewing for second scan onwards
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
    frames = this->integrateImu(this->prev_scan_stamp, this->state.q, this->state.p,
                                this->state.v, timestamps);
    // this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

    // TO DO: if there are no frames between the start and end of the sweep
    // that probably means that there's a sync issue
    // if (frames.size() != timestamps.size()) {
    //     ROS_FATAL("Bad time sync between LiDAR and IMU!");

    //     this->T_prior = this->T;
    //     pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extr.baselink2lidar_T);
    //     this->deskewed_scan = deskewed_scan_;
    //     this->deskew_status = false;
    //     return;
    // }

    // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
    this->T_prior = frames[median_pt_index];

    #pragma omp parallel for num_threads(this->num_threads_)
    for (int i = 0; i < timestamps.size(); i++) {

        Eigen::Matrix4f T = frames[i] * this->extr.baselink2lidar_T;

        // transform point to world frame
        // TO DO: deskewed scan must be in lidar frame --> taking into account last frame (maybe pick frames from iKFoM)
        for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
        auto &pt = deskewed_scan_->points[k];
        pt.getVector4fMap()[3] = 1.;
        pt.getVector4fMap() = T * pt.getVector4fMap();
        }
    }

    // this->deskew_status = true;

    return deskewed_scan_; // should be in global/world frame
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
Localizer::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                        Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps){

    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

    if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
        // invalid input, return empty vector
        return empty;
    }

    boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it;
    boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it;
    if (not this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it)) {
        // not enough IMU measurements, return empty vector
        return empty;
    }

    // Backwards integration to find pose at first IMU sample
    const IMUmeas& f1 = *begin_imu_it;
    const IMUmeas& f2 = *(begin_imu_it+1);

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