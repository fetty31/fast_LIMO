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

#ifndef __FASTLIMO_LOCALIZER_HPP__
#define __FASTLIMO_LOCALIZER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/Plane.hpp"
#include "fast_limo/Utils/Config.hpp"
#include "fast_limo/Utils/Algorithms.hpp"

using namespace fast_limo;

class fast_limo::Localizer {

    // VARIABLES

    public:
        pcl::PointCloud<PointType>::Ptr pc2match; // pointcloud to match in Xt2 (last_state) frame

    private:
        // Iterated Kalman Filter on Manifolds (FASTLIOv2)
        esekfom::esekf<state_ikfom, 12, input_ikfom> _iKFoM;
        std::mutex mtx_ikfom;

        State state, last_state;
        Extrinsics extr;
        SensorType sensor;
        IMUmeas last_imu;

        // Config struct
        Config config;

        // Matches (debug aux var.)
        Matches matches;

        // PCL Filters
        pcl::CropBox<PointType> crop_filter;
        pcl::VoxelGrid<PointType> voxel_filter;

        // Point Clouds
        pcl::PointCloud<PointType>::ConstPtr original_scan; // in base_link/body frame
        pcl::PointCloud<PointType>::ConstPtr deskewed_scan; // in global/world frame
        pcl::PointCloud<PointType>::Ptr final_raw_scan;     // in global/world frame
        pcl::PointCloud<PointType>::Ptr final_scan;         // in global/world frame

        // Time related var.
        double scan_stamp;
        double prev_scan_stamp;
        double scan_dt;

        double imu_stamp;
        double prev_imu_stamp;
        double imu_dt;
        double first_imu_stamp;
        double last_propagate_time_;
        double imu_calib_time_;

        // Gravity
        double gravity_;

        // Flags
        bool imu_calibrated_;

        // OpenMP max threads
        int num_threads_;

        // IMU buffer
        boost::circular_buffer<IMUmeas> imu_buffer;

        // Propagated states buffer
        boost::circular_buffer<State> propagated_buffer;
        std::mutex mtx_prop; // mutex for avoiding multiple thread access to the buffer
        std::condition_variable cv_prop_stamp;


        // IMU axis matrix 
        Eigen::Matrix3f imu_accel_sm_;
        /*(if your IMU doesn't comply with axis system ISO-8855, 
        this matrix is meant to map its current orientation with respect to the standard axis system)
            Y-pitch
            ^   
            |  
            | 
            |
      Z-yaw o-----------> X-roll
        */

        // Debugging
        unsigned char calibrating = 0;

            // Threads
        std::thread debug_thread;

            // Buffers
        boost::circular_buffer<double> cpu_times;
        boost::circular_buffer<double> imu_rates;
        boost::circular_buffer<double> lidar_rates;
        boost::circular_buffer<double> cpu_percents;

            // CPU specs
        std::string cpu_type;
        clock_t lastCPU, lastSysCPU, lastUserCPU;
        int numProcessors;

            // CPU stats
        std::mutex mtx_cpu_stats;
        float cpu_time;
        float cpu_max_time;
        float cpu_mean_time;
        float cpu_cores, cpu_load, cpu_max_load;
        float ram_usage;

            // Other
        std::chrono::duration<double> elapsed_time;  // pointcloud callback elapsed time
        int deskew_size;                        // steps taken to deskew (FoV discretization)
        int propagated_size;                    // number of integrated states

    // FUNCTIONS

    public:
        Localizer();
        void init(Config& cfg);

        // Callbacks 
        void updateIMU(IMUmeas& raw_imu);
        void updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp);

        // Get output
        pcl::PointCloud<PointType>::Ptr get_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_finalraw_pointcloud();

        pcl::PointCloud<PointType>::ConstPtr get_orig_pointcloud();
        pcl::PointCloud<PointType>::ConstPtr get_deskewed_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_pc2match_pointcloud();

        Matches& get_matches();

        State getWorldState();  // get state in body/base_link frame
        State getBodyState();   // get state in LiDAR frame

        std::vector<double> getPoseCovariance(); // get eKF covariances
        std::vector<double> getTwistCovariance();// get eKF covariances
        
        double get_propagate_time();

        // CPU stats
        void get_cpu_stats(float &comput_time, float &max_comput_time, float &mean_comput_time,
                            float &cpu_cores, float &cpu_load, float &cpu_max_load, float &ram_usage);

        // Status info
        bool is_calibrated();

        // Config
        void set_sensor_type(uint8_t type);
        fast_limo::SensorType get_sensor_type();

        // iKFoM measurement model
        void calculate_H(const state_ikfom&, const Matches&, Eigen::MatrixXd& H, Eigen::VectorXd& h);

        // Backpropagation
        void propagateImu(const IMUmeas& imu);
        void propagateImu(double t1, double t2);

    private:
        void init_iKFoM();
        void init_iKFoM_state();

        IMUmeas imu2baselink(IMUmeas& imu);

        pcl::PointCloud<PointType>::Ptr deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time);

        States integrateImu(double start_time, double end_time, State& state);

        bool propagatedFromTimeRange(double start_time, double end_time,
                                  boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
                                  boost::circular_buffer<State>::reverse_iterator& end_prop_it);
        bool imuMeasFromTimeRange(double start_time, double end_time,
                                  boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
                                  boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it);
        bool isInRange(PointType& p);

        void getCPUinfo();
        void debugVerbose();

    // SINGLETON 

    public:
        static Localizer& getInstance(){
            static Localizer* loc = new Localizer();
            return *loc;
        }

    private:
        // Disable copy/move functionality
        Localizer(const Localizer&) = delete;
        Localizer& operator=(const Localizer&) = delete;
        Localizer(Localizer&&) = delete;
        Localizer& operator=(Localizer&&) = delete;

};

#endif
