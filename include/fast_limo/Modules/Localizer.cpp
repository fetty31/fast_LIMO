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


using namespace fast_limo;
// class fast_limo::Localizer
	// public

Localizer::Localizer() : scan_stamp_(0.0),
                         prev_scan_stamp_(0.0),
												 deskew_size_(0),
												 propagated_size_(0),
												 numProcessors_(0),
												 imu_stamp_(0.0),
												 prev_imu_stamp_(0.0),
												 first_imu_stamp_(0.0),
												 gravity_(9.81),
												 imu_calibrated_(false) { 

	original_scan_  = PointCloudT::ConstPtr (boost::make_shared<PointCloudT>());
	deskewed_scan_  = PointCloudT::ConstPtr (boost::make_shared<PointCloudT>());
	pc2match_       = PointCloudT::Ptr (boost::make_shared<PointCloudT>());
	final_raw_scan_ = PointCloudT::Ptr (boost::make_shared<PointCloudT>());
	final_scan_     = PointCloudT::Ptr (boost::make_shared<PointCloudT>());
}

void Localizer::init() {

	Config& config = Config::getInstance();

	// Update Mapper config
	Mapper& map = Mapper::getInstance();

	// Initialize Iterated Kalman Filter on Manifolds
	init_iKFoM();

	// Set buffer capacity
	imu_buffer_.set_capacity(2000);
	propagated_buffer_.set_capacity(2000);

	// PCL filters setup
  crop_filter_.setNegative(true);
	crop_filter_.setMin(config.filters.cropBoxMin); 
	crop_filter_.setMax(config.filters.cropBoxMax);

	voxel_filter_.setLeafSize(config.filters.leafSize);
		
	// IMU intrinsics
	state_.b.accel = config.intrinsics.accel_bias;
	state_.b.gyro  = config.intrinsics.gyro_bias;

	// Avoid unnecessary warnings from PCL
	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

	// Initial calibration
	if (not (config.gravity_align || config.calibrate_accel || config.calibrate_gyro)) {
	  imu_calibrated_ = true;
		init_iKFoM_state();
	}

	// CPU info
	getCPUinfo();

	if (config.verbose) {
		// set up buffer capacities
		imu_rates_.set_capacity(1000);
		lidar_rates_.set_capacity(1000);
		cpu_times_.set_capacity(1000);
		cpu_percents_.set_capacity(1000);
	}
}


void Localizer::updatePointCloud(PointCloudT::Ptr& raw_pc, double time_stamp) {
	Config& config = Config::getInstance();

	auto start_time = chrono::system_clock::now();

	if (raw_pc->points.size() < 1) {
		std::cout << "FAST_LIMO::Raw PointCloud is empty!\n";
		return;
	}

	if (not is_calibrated())
		return;

	if (imu_buffer_.empty()) {
		std::cout << "FAST_LIMO::IMU buffer is empty!\n";
		return;
	}

	// Remove NaNs
	std::vector<int> idx;
	raw_pc->is_dense = false;
	pcl::removeNaNFromPointCloud(*raw_pc, *raw_pc, idx);

	// Crop Box Filter (1 m^2)
	if (config.filters.crop_active) {
		crop_filter_.setInputCloud(raw_pc);
		crop_filter_.filter(*raw_pc);
	}

	// Distance & Time Rate filters
	PointCloudT::Ptr input_pc(boost::make_shared<PointCloudT>());

	for (int i = 0; i < raw_pc->size(); i++) {
	  PointType p = raw_pc->points[i];
		
		if (config.filters.dist_active) {
			if (Eigen::Vector3f(p.x, p.y, p.z).norm() <= config.filters.min_dist)
				continue;
		}

		if (config.filters.rate_active) {
			if (i % config.filters.rate_value != 0)
				continue;
		}

		if (config.filters.fov_active) {
			if (fabs(atan2(p.y, p.x)) < config.filters.fov_angle)
				continue;
		}

		input_pc->push_back(p);
	}

	if (config.debug)
		original_scan_ = input_pc;

	// Motion compensation
	PointCloudT::Ptr deskewed_Xt2_pc(boost::make_shared<PointCloudT>());
	deskewed_Xt2_pc = deskewPointCloud(input_pc, time_stamp);

	// Voxel Grid Filter
	if (config.filters.voxel_active) { 
		voxel_filter_.setInputCloud(deskewed_Xt2_pc);
		voxel_filter_.filter(*pc2match_);
	} else {
		pc2match_ = deskewed_Xt2_pc;
	}

	if (pc2match_->points.size() > 1) {

	mtx_ikfom_.lock(); // Lock iKFoM

		double solve_time = 0.0;
		iKFoM_.update_iterated_dyn_share_modified(0.001 /*LiDAR noise*/);
		
		state_    = State(iKFoM_.get_x());
		state_.w  = last_imu_.ang_vel;
		state_.a  = last_imu_.lin_accel;

	mtx_ikfom_.unlock();

		pcl::transformPointCloud(*pc2match_, *final_scan_,
		                         state_.get_RT() * state_.get_extr_RT());

		if (config.debug)
			pcl::transformPointCloud(*deskewed_Xt2_pc, *final_raw_scan_,
			                         state_.get_RT() * state_.get_extr_RT());

		// Add scan to map
		fast_limo::Mapper& map = fast_limo::Mapper::getInstance();
		map.add(final_scan_, scan_stamp_);

	} else {
		std::cout << "-------------- FAST_LIMO::NULL ITERATION --------------\n";
	}

	auto end_time = chrono::system_clock::now();
	elapsed_time_ = end_time - start_time;

	if (config.verbose) {
		if (prev_scan_stamp_ > 0.0)
			lidar_rates_.push_front(1. / (scan_stamp_ - prev_scan_stamp_));

		if (calibrating_ > 0)
			cpu_times_.push_front(elapsed_time_.count());
		else
		  cpu_times_.push_front(0.0);
		
		if (calibrating_ < UCHAR_MAX)
			calibrating_++;

		// debug thread
		debug_thread_ = std::thread(&Localizer::debugVerbose, this);
		debug_thread_.detach();
	}

	prev_scan_stamp_ = scan_stamp_;
}

void Localizer::updateIMU(IMUmeas& imu) {

	Config& config = Config::getInstance();

	imu_stamp_ = imu.stamp;
	imu.dt = imu.stamp - prev_imu_stamp_;
	
	if ( (imu.dt == 0.) || (imu.dt > 0.1) ) { imu.dt = 1.0/400.0; }


	if (first_imu_stamp_ == 0.0)
		first_imu_stamp_ = imu.stamp;
	

	if (config.verbose) 
			imu_rates_.push_front( 1./imu.dt );

	// IMU calibration procedure - do only while the robot is in stand still!
	if (not imu_calibrated_) {

		static int num_samples = 0;
		static Eigen::Vector3f gyro_avg(0., 0., 0.);
		static Eigen::Vector3f accel_avg(0., 0., 0.);
		static bool print = true;

		if ((imu.stamp - first_imu_stamp_) < config.imu_calib_time) {

			num_samples++;

			gyro_avg[0] += imu.ang_vel[0];
			gyro_avg[1] += imu.ang_vel[1];
			gyro_avg[2] += imu.ang_vel[2];

			accel_avg[0] += imu.lin_accel[0];
			accel_avg[1] += imu.lin_accel[1];
			accel_avg[2] += imu.lin_accel[2];

			if (print) {
				std::cout << std::endl 
				          << " Calibrating IMU for "
									<< config.imu_calib_time
									<< " seconds... \n";

				std::cout.flush();
				print = false;
			}

		} else {

			gyro_avg /= num_samples;
			accel_avg /= num_samples;

			Eigen::Vector3f grav_vec (0., 0., gravity_);

			state_.q = imu.q;

			if (config.gravity_align) {

				std::cout << " Accel mean: [ " 
				          << accel_avg[0] << ", " 
									<< accel_avg[1] << ", " 
									<< accel_avg[2] << " ]\n";

				// Estimate gravity vector - Only approximate if biases have not been pre-calibrated
				grav_vec = (accel_avg - state_.b.accel).normalized() * abs(gravity_);
				Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., gravity_));
				
				std::cout << " Gravity mean: [ "
				          << grav_vec[0] << ", "
									<< grav_vec[1] << ", "
									<< grav_vec[2] << " ]\n";

				// set gravity aligned orientation
				state_.q = grav_q;

			}

			if (config.calibrate_accel) {

				// subtract gravity from avg accel to get bias
				state_.b.accel = accel_avg - grav_vec;

				std::cout << std::setprecision(8) << " Accel biases [xyz]: " 
									<< state_.b.accel[0] << ", "
									<< state_.b.accel[1] << ", "
									<< state_.b.accel[2] << std::endl;
			}

			if (config.calibrate_gyro) {

				state_.b.gyro = gyro_avg;

				std::cout << std::setprecision(8) << " Gyro biases  [xyz]: "
				          << state_.b.gyro[0] << ", "
				          << state_.b.gyro[1] << ", "
				          << state_.b.gyro[2] << std::endl;
			}

			state_.q.normalize();

			// Set initial KF state
			init_iKFoM_state();

			// Set calib flag
			imu_calibrated_ = true;

			// Initial attitude
			auto euler = state_.q.toRotationMatrix().eulerAngles(2, 1, 0);
			double yaw   = euler[0] * (180.0/M_PI);
			double pitch = euler[1] * (180.0/M_PI);
			double roll  = euler[2] * (180.0/M_PI);

			// use alternate representation if the yaw is smaller
			if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
				yaw   = remainder(yaw + 180.0,   360.0);
				pitch = remainder(180.0 - pitch, 360.0);
				roll  = remainder(roll + 180.0,  360.0);
			}

			std::cout << std::setprecision(4) << " Estimated initial attitude:" << std::endl;
			std::cout << "   Roll  [deg]: " << roll << std::endl;
			std::cout << "   Pitch [deg]: " << pitch << std::endl;
			std::cout << "   Yaw   [deg]: " << yaw << std::endl;
			std::cout << std::endl;
		}

	} else {

		// Apply the calibrated bias to the new IMU measurements
		Eigen::Vector3f lin_accel_corrected = (config.intrinsics.imu_sm * imu.lin_accel) 
		                                       - state_.b.accel;
		Eigen::Vector3f ang_vel_corrected = imu.ang_vel - state_.b.gyro;

		imu.lin_accel = lin_accel_corrected;
		imu.ang_vel   = ang_vel_corrected;

		last_imu_ = imu;
		prev_imu_stamp_ = imu.stamp;

		// Store calibrated IMU measurements into imu buffer for manual integration later.
		imu_buffer_.push_front(imu);

		// iKFoM propagate state
		propagateImu(imu);
		cv_prop_stamp_.notify_one(); // Notify PointCloud thread that propagated IMU data exists for this time
	}
}

void Localizer::propagateImu(const IMUmeas& imu){
	Config& config = Config::getInstance();

	input_ikfom in;
	in.acc = imu.lin_accel.cast<double>();
	in.gyro = imu.ang_vel.cast<double>();

	Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
	Q.block<3, 3>(0, 0) = config.ikfom.cov_gyro * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(3, 3) = config.ikfom.cov_acc * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(6, 6) = config.ikfom.cov_bias_gyro * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(9, 9) = config.ikfom.cov_bias_acc * Eigen::Matrix3d::Identity();

	// Propagate IMU measurement
	double dt = imu.dt;

mtx_ikfom_.lock();
	iKFoM_.predict(dt, Q, in);
mtx_ikfom_.unlock();

	// Save propagated state for motion compensation
mtx_prop_.lock();
	propagated_buffer_.push_front(
				State(iKFoM_.get_x(), imu.stamp, imu.lin_accel, imu.ang_vel));
mtx_prop_.unlock();

}



void Localizer::init_iKFoM() {
	Config& config = Config::getInstance();

	// Initialize IKFoM
	iKFoM_.init_dyn_share(
		&Localizer::get_f_wrapper,
		&Localizer::df_dx_wrapper,
		&Localizer::df_dw_wrapper,
		&Localizer::h_share_model_wrapper,
		config.ikfom.MAX_NUM_ITERS,
		config.ikfom.LIMITS
	);
}

void Localizer::init_iKFoM_state() {
	Config& config = Config::getInstance();

	state_ikfom init_state = iKFoM_.get_x();
	init_state.rot = state_.q.cast<double>();
	init_state.pos = state_.p.cast<double>();
	init_state.grav = S2(Eigen::Vector3d(0., 0., -gravity_));
	init_state.bg = state_.b.gyro.cast<double>();
	init_state.ba = state_.b.accel.cast<double>();

	init_state.offset_R_L_I = SO3(config.extrinsics.lidar2baselink_T.linear().cast<double>());
	init_state.offset_T_L_I = config.extrinsics.lidar2baselink_T.translation().cast<double>();
	iKFoM_.change_x(init_state); // set initial state

	esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = iKFoM_.get_P();
	init_P.setIdentity();
	init_P *= 1e-8f;
	
	iKFoM_.change_P(init_P);
}


PointCloudT::Ptr Localizer::deskewPointCloud(PointCloudT::Ptr& pc, double& start_time){

	Config& config = Config::getInstance();

	if (pc->points.size() < 1) 
		return boost::make_shared<PointCloudT>();

	// individual point timestamps should be relative to this time
	double sweep_ref_time = start_time;
	bool end_of_sweep = config.end_of_sweep;

	// sort points by timestamp
	std::function<bool(const PointType&, const PointType&)> point_time_cmp;
	std::function<double(PointType&)> extract_point_time;

	if (config.sensor_type == (int)fast_limo::SensorType::OUSTER) {

		point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
		{   if (end_of_sweep) return p1.t > p2.t; 
			else return p1.t < p2.t; };
		extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
		{   if (end_of_sweep) return sweep_ref_time - pt.t * 1e-9f; 
			else return sweep_ref_time + pt.t * 1e-9f; };

	} else if (config.sensor_type == (int)fast_limo::SensorType::VELODYNE) {
		
		point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
		{   if (end_of_sweep) return p1.time > p2.time; 
			else return p1.time < p2.time; };
		extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
		{   if (end_of_sweep) return sweep_ref_time - pt.time; 
			else return sweep_ref_time + pt.time; };

	} else if (config.sensor_type == (int)fast_limo::SensorType::HESAI) {

		point_time_cmp = [](const PointType& p1, const PointType& p2)
		{ return p1.timestamp < p2.timestamp; };
		extract_point_time = [](PointType& pt)
		{ return pt.timestamp; };

	} else if (config.sensor_type == (int)fast_limo::SensorType::LIVOX) {
		
		point_time_cmp = [](const PointType& p1, const PointType& p2)
		{ return p1.timestamp < p2.timestamp; };
		extract_point_time = [](PointType& pt)
		{ return pt.timestamp * 1e-9f; };

	} else {
		std::cout << "-------------------------------------------------------------------\n";
		std::cout << "FAST_LIMO::FATAL ERROR: LiDAR sensor type unknown or not specified!\n";
		std::cout << "-------------------------------------------------------------------\n";
		return boost::make_shared<PointCloudT>();
	}

	// copy points into deskewed_scan_ in order of timestamp
	PointCloudT::Ptr deskewed_scan_(boost::make_shared<PointCloudT>());
	deskewed_scan_->points.resize(pc->points.size());
	
	std::partial_sort_copy(pc->points.begin(), pc->points.end(),
							           deskewed_scan_->points.begin(), deskewed_scan_->points.end(),
												 point_time_cmp);

	if (deskewed_scan_->points.size() < 1){
		std::cout << "FAST_LIMO::ERROR: failed to sort input pointcloud!\n";
		return boost::make_shared<PointCloudT>();
	}

	// compute offset between sweep reference time and IMU data
	double offset = 0.0;
	if (config.time_offset) {
		offset = imu_stamp_ - extract_point_time(deskewed_scan_->points.back()) - 1.e-4; // automatic sync (not precise!)
		if (offset > 0.0) offset = 0.0; // don't jump into future
	}

	// Set scan_stamp for next iteration
	scan_stamp_ = extract_point_time(deskewed_scan_->points.back()) + offset;

	// IMU prior & deskewing 
	// if (prev_scan_stamp_ <= 0.) 
		// prev_scan_stamp_ = scan_stamp_ - 0.1;
	States frames = integrateImu(prev_scan_stamp_, scan_stamp_); // baselink/body frames

	if (frames.size() < 1) {
		std::cout << "FAST_LIMO::ERROR: No frames obtained from IMU propagation!\n";
		std::cout << "           Returning null deskewed pointcloud!\n";
		return boost::make_shared<PointCloudT>();
	}

	// deskewed pointcloud w.r.t last known state prediction
	PointCloudT::Ptr deskewed_Xt2_scan(boost::make_shared<PointCloudT>());
	deskewed_Xt2_scan->points.resize(deskewed_scan_->points.size());

	last_state_ = fast_limo::State(iKFoM_.get_x()); // baselink/body frame

	#pragma omp parallel for num_threads(config.num_threads)
	for (int k = 0; k < deskewed_scan_->points.size(); k++) {

		int i_f = algorithms::binary_search_tailored(frames, extract_point_time(deskewed_scan_->points[k])+offset);

		State X0 = frames[i_f];
		X0.update(extract_point_time(deskewed_scan_->points[k]) + offset);

		Eigen::Matrix4f T = X0.get_RT() * X0.get_extr_RT();

		// world frame deskewed pc
		auto pt = deskewed_scan_->points[k]; // lidar frame
		pt.getVector4fMap()[3] = 1.;
		pt.getVector4fMap() = T * pt.getVector4fMap(); // world/global frame

		// Xt2 frame deskewed pc
		auto pt2 = deskewed_scan_->points[k];
		pt2.getVector4fMap() = last_state_.get_extr_RT_inv() * last_state_.get_RT_inv() * pt.getVector4fMap(); // Xt2 frame
		pt2.intensity = pt.intensity;

		deskewed_Xt2_scan->points[k] = pt2;
	}

	// debug info
	deskew_size_ = deskewed_Xt2_scan->points.size(); 
	propagated_size_ = frames.size();

	if (config.debug && deskew_size_ > 0) // debug only
		deskewed_scan_ = deskewed_Xt2_scan;

	return deskewed_Xt2_scan; 
}


States Localizer::integrateImu(double start_time, double end_time) {

	States imu_se3;

	boost::circular_buffer<State>::reverse_iterator begin_prop_it;
	boost::circular_buffer<State>::reverse_iterator end_prop_it;
	if (not propagatedFromTimeRange(start_time, end_time, begin_prop_it, end_prop_it)) {
		// not enough IMU measurements, return empty vector
		std::cout << "FAST_LIMO::propagatedFromTimeRange(): not enough propagated states!\n";
		return imu_se3;
	}

	for(auto it = begin_prop_it; it != end_prop_it; it++)
		imu_se3.push_back(*it);

	return imu_se3;
}

bool Localizer::propagatedFromTimeRange(double start_time, double end_time,
										boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
										boost::circular_buffer<State>::reverse_iterator& end_prop_it) {

	if (propagated_buffer_.empty() || propagated_buffer_.front().time < end_time) {
		// Wait for the latest IMU data
		std::cout << "PROPAGATE WAITING...\n";
		std::cout << "     - buffer time: " << propagated_buffer_.front().time << std::endl;
		std::cout << "     - end scan time: " << end_time << std::endl;
		std::unique_lock<decltype(mtx_prop_)> lock(mtx_prop_);
		cv_prop_stamp_.wait(lock, [this, &end_time] { 
				return propagated_buffer_.front().time >= end_time; });
	}

	auto prop_it = propagated_buffer_.begin();

	auto last_prop_it = prop_it;
	prop_it++;
	while (prop_it != propagated_buffer_.end() && prop_it->time >= end_time) {
		last_prop_it = prop_it;
		prop_it++;
	}

	while (prop_it != propagated_buffer_.end() && prop_it->time >= start_time) {
		prop_it++;
	}

	if (prop_it == propagated_buffer_.end()) {
		// not enough IMU measurements, return false
		return false;
	}

	prop_it++;

	// Set reverse iterators (to iterate forward in time)
	end_prop_it = boost::circular_buffer<State>::reverse_iterator(last_prop_it);
	begin_prop_it = boost::circular_buffer<State>::reverse_iterator(prop_it);

	return true;
}


void Localizer::h_share_model(state_ikfom &updated_state,
							  esekfom::dyn_share_datastruct<double> &ekfom_data) {

	Config& config = Config::getInstance();

  Mapper& MAP = Mapper::getInstance();
	if (not MAP.exists()) {
		ekfom_data.h_x = Eigen::MatrixXd::Zero(0, 12);
		ekfom_data.h.resize(0);
		return;
	}

	int N = pc2match_->size();

	std::vector<bool> chosen(N, false);
	std::vector<Match> matches(N);

	State S(updated_state);

	#pragma omp parallel for num_threads(5)
	for (int i = 0; i < N; i++) {
		auto p = pc2match_->points[i];
		Eigen::Vector4f bl4_point(p.x, p.y, p.z, 1.);
		Eigen::Vector4f g = (S.get_RT() * S.get_extr_RT()) * bl4_point; 

		MapPoints near_points;
		std::vector<float> pointSearchSqDis(config.ikfom.mapping.NUM_MATCH_POINTS);
		MAP.knn(MapPoint(g(0), g(1), g(2)), 
		        config.ikfom.mapping.NUM_MATCH_POINTS,
						near_points,
						pointSearchSqDis);
		
		if (near_points.size() < config.ikfom.mapping.NUM_MATCH_POINTS 
		    or pointSearchSqDis.back() > 2)
					continue;
		
		Eigen::Vector4f p_abcd = Eigen::Vector4f::Zero();
		if (not algorithms::estimate_plane(p_abcd, near_points, config.ikfom.mapping.PLANE_THRESHOLD))
			continue;
		
		chosen[i] = true;
		matches[i] = std::make_pair(g, p_abcd);
	}


	std::vector<Match> clean_matches;
	for (int i = 0; i < N; i++) {
		if (chosen[i])
			clean_matches.push_back(matches[i]);
	}

	ekfom_data.h_x = Eigen::MatrixXd::Zero(clean_matches.size(), 12);
	ekfom_data.h.resize(clean_matches.size());	

	// For each match, calculate its derivative and distance
	#pragma omp parallel for num_threads(8)
	for (int i = 0; i < clean_matches.size(); ++i) {
		Match match = clean_matches[i];
		Eigen::Vector4f p4_imu   = S.get_RT_inv() * match.first;
		Eigen::Vector4f p4_lidar = S.get_extr_RT_inv() * p4_imu;
		Eigen::Vector4f normal   = match.second;

		// Rotation matrices
		Eigen::Matrix3f R_inv = updated_state.rot.conjugate().toRotationMatrix().cast<float>();
		Eigen::Matrix3f I_R_L_inv = updated_state.offset_R_L_I.conjugate().toRotationMatrix().cast<float>();

		// Set correct dimensions
		Eigen::Vector3f p_lidar, p_imu, n;
		p_lidar = p4_lidar.head(3);
		p_imu   = p4_imu.head(3);
		n       = normal.head(3);

		// Calculate measurement Jacobian H (:= dh/dx)
		Eigen::Vector3f C = R_inv * n;
		Eigen::Vector3f B = p_lidar.cross(I_R_L_inv * C);
		Eigen::Vector3f A = p_imu.cross(C);
		
		ekfom_data.h_x.block<1, 6>(i,0) << n(0), n(1), n(2), A(0), A(1), A(2);
		if (config.ikfom.estimate_extrinsics)
			ekfom_data.h_x.block<1, 6>(i,6) << B(0), B(1), B(2), C(0), C(1), C(2);

		// Measurement: distance to the closest plane
		double dist = normal(0) * match.first(0) 
		              + normal(1) * match.first(1)
									+ normal(2) * match.first(2)
									+ normal(3); 

		ekfom_data.h(i) = -dist;
	}

}

Eigen::Matrix<double, 24, 1> Localizer::get_f(state_ikfom &s,
											  const input_ikfom &in) {

  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc-s.ba);
  for(int i = 0; i < 3; i++ ){
	res(i) = s.vel[i];
	res(i + 3) =  omega[i]; 
	res(i + 12) = a_inertial[i] + s.grav[i]; 
  }
  return res;
}

Eigen::Matrix<double, 24, 23> Localizer::df_dx(state_ikfom &s, const input_ikfom &in) {

  Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
  cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
  Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 2>(12, 21) =  grav_matrix;
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, 24, 12> Localizer::df_dw(state_ikfom &s, const input_ikfom &in) {

  Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
  cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
  cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return cov;
}


// #############################################################################
// ################################ GETTERS ####################################
// #############################################################################


PointCloudT::Ptr Localizer::get_pointcloud() { return final_scan_; }

PointCloudT::Ptr Localizer::get_finalraw_pointcloud()  { return final_raw_scan_; }

PointCloudT::ConstPtr Localizer::get_orig_pointcloud() { return original_scan_; }

PointCloudT::ConstPtr Localizer::get_deskewed_pointcloud() { return deskewed_scan_; }

PointCloudT::Ptr Localizer::get_pc2match_pointcloud() { return pc2match_; }

bool Localizer::is_calibrated() { return imu_calibrated_; }

State Localizer::getBodyState() {

	if (not is_calibrated())
		return State();

	State out = iKFoM_.get_x();

	out.w    = last_imu_.ang_vel;
	out.a    = last_imu_.lin_accel;
	out.time = imu_stamp_; 

	out.p += out.pLI;
	out.q *= out.qLI;
	out.v = out.q.toRotationMatrix().transpose() * out.v;

	return out;
}

State Localizer::getWorldState() {

	if (not is_calibrated())
		return State();

	State out = iKFoM_.get_x();

	out.w    = last_imu_.ang_vel;
	out.a    = last_imu_.lin_accel;
	out.time = imu_stamp_;

	out.v = out.q.toRotationMatrix().transpose() * out.v;

	return out;
}

std::vector<double> Localizer::getPoseCovariance() {
	if (not is_calibrated())
		return std::vector<double>(36, 0);

	esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P = iKFoM_.get_P();
	Eigen::Matrix<double, 6, 6> P_pose;
	P_pose.block<3, 3>(0, 0) = P.block<3, 3>(3, 3);
	P_pose.block<3, 3>(0, 3) = P.block<3, 3>(3, 0);
	P_pose.block<3, 3>(3, 0) = P.block<3, 3>(0, 3);
	P_pose.block<3, 3>(3, 3) = P.block<3, 3>(0, 0);

	std::vector<double> cov(P_pose.size());
	Eigen::Map<Eigen::MatrixXd>(cov.data(), P_pose.rows(), P_pose.cols()) = P_pose;

	return cov;
}

std::vector<double> Localizer::getTwistCovariance() {
	Config& config = Config::getInstance();

	if (not is_calibrated())
		return std::vector<double>(36, 0);

	esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P = iKFoM_.get_P();
	Eigen::Matrix<double, 6, 6> P_odom = Eigen::Matrix<double, 6, 6>::Zero();
	P_odom.block<3, 3>(0, 0) = P.block<3, 3>(6, 6);
	P_odom.block<3, 3>(3, 3) = config.ikfom.cov_gyro * Eigen::Matrix3d::Identity();

	std::vector<double> cov(P_odom.size());
	Eigen::Map<Eigen::MatrixXd>(cov.data(), P_odom.rows(), P_odom.cols()) = P_odom;

	return cov;
}

























































void Localizer::getCPUinfo(){ // CPU Specs
	char CPUBrandString[0x40];
	memset(CPUBrandString, 0, sizeof(CPUBrandString));

	cpu_type_ = "";

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
	cpu_type_ = CPUBrandString;
	boost::trim(cpu_type_);
	#endif

	FILE* file;
	struct tms timeSample;
	char line[128];

	lastCPU_ = times(&timeSample);
	lastSysCPU_ = timeSample.tms_stime;
	lastUserCPU_ = timeSample.tms_utime;

	file = fopen("/proc/cpuinfo", "r");
	numProcessors_ = 0;
	while(fgets(line, 128, file) != nullptr) {
		if (strncmp(line, "processor", 9) == 0)
			numProcessors_++;
	}
	fclose(file);
}

void Localizer::debugVerbose(){

	Config& config = Config::getInstance();

	// Average computation time
	double avg_comp_time =
		std::accumulate(cpu_times_.begin(), cpu_times_.end(), 0.0) / cpu_times_.size();

	// Average sensor rates
	double avg_imu_rate =
		std::accumulate(imu_rates_.begin(), imu_rates_.end(), 0.0) / imu_rates_.size();
	double avg_lidar_rate =
		std::accumulate(lidar_rates_.begin(), lidar_rates_.end(), 0.0) / lidar_rates_.size();

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
	if (now <= lastCPU_ || timeSample.tms_stime < lastSysCPU_ ||
		timeSample.tms_utime < lastUserCPU_) {
		cpu_percent = -1.0;
	} else {
		cpu_percent = (timeSample.tms_stime - lastSysCPU_) + (timeSample.tms_utime - lastUserCPU_);
		cpu_percent /= (now - lastCPU_);
		cpu_percent /= numProcessors_;
		cpu_percent *= 100.;
	}
	lastCPU_ = now;
	lastSysCPU_ = timeSample.tms_stime;
	lastUserCPU_ = timeSample.tms_utime;
	cpu_percents_.push_front(cpu_percent);
	double avg_cpu_usage =
		std::accumulate(cpu_percents_.begin(), cpu_percents_.end(), 0.0) / cpu_percents_.size();

	// ------------------------------------- PRINT OUT -------------------------------------

	printf("\033[2J\033[1;1H");
	std::cout << std::endl
				<< "+-------------------------------------------------------------------+" << std::endl;
	std::cout   << "|                        Fast LIMO  v" << FAST_LIMO_v  << "                          |"
				<< std::endl;
	std::cout   << "+-------------------------------------------------------------------+" << std::endl;

	std::time_t curr_time = scan_stamp_;
	std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
	std::cout << "| " << std::left << asc_time;
	std::cout << std::right << std::setfill(' ') << std::setw(42)
		<< "Elapsed Time: " + to_string_with_precision(imu_stamp_ - first_imu_stamp_, 2) + " seconds "
		<< "|" << std::endl;

	if ( !cpu_type_.empty() ) {
		std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< cpu_type_ + " x " + std::to_string(numProcessors_)
		<< "|" << std::endl;
	}

	if (config.sensor_type == (int)fast_limo::SensorType::OUSTER) {
		std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
									+ " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
		<< "|" << std::endl;
	} else if (config.sensor_type == (int)fast_limo::SensorType::VELODYNE) {
		std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
										+ " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
		<< "|" << std::endl;
	} else if (config.sensor_type == (int)fast_limo::SensorType::HESAI) {
		std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
									+ " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
		<< "|" << std::endl;
	} else if (config.sensor_type == (int)fast_limo::SensorType::LIVOX) {
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

	State final_state = getWorldState();

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
		<< "Deskewed points: " + std::to_string(deskew_size_) << "|" << std::endl;
	std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< "Integrated states: " + std::to_string(propagated_size_) << "|" << std::endl;
	std::cout << "|                                                                   |" << std::endl;

	std::cout << std::right << std::setprecision(2) << std::fixed;
	std::cout << "| Computation Time :: "
		<< std::setfill(' ') << std::setw(6) << cpu_times_.front()*1000. << " ms    // Avg: "
		<< std::setw(6) << avg_comp_time*1000. << " / Max: "
		<< std::setw(6) << *std::max_element(cpu_times_.begin(), cpu_times_.end())*1000.
		<< "     |" << std::endl;
	std::cout << "| Cores Utilized   :: "
		<< std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * numProcessors_ << " cores // Avg: "
		<< std::setw(6) << (avg_cpu_usage/100.) * numProcessors_ << " / Max: "
		<< std::setw(6) << (*std::max_element(cpu_percents_.begin(), cpu_percents_.end()) / 100.)
						* numProcessors_
		<< "     |" << std::endl;
	std::cout << "| CPU Load         :: "
		<< std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
		<< std::setw(6) << avg_cpu_usage << " / Max: "
		<< std::setw(6) << *std::max_element(cpu_percents_.begin(), cpu_percents_.end())
		<< "     |" << std::endl;
	std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
		<< "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
		<< "|" << std::endl;

	std::cout << "+-------------------------------------------------------------------+" << std::endl;

}