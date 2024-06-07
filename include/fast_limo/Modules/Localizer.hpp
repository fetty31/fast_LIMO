#ifndef __FASTLIMO_LOCALIZER_HPP__
#define __FASTLIMO_LOCALIZER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/Plane.hpp"

using namespace fast_limo;

class fast_limo::Localizer {

    // VARIABLES

    public:
        pcl::PointCloud<PointType>::ConstPtr pc2match; // pointcloud to match in Xt2 (last_state) frame

    private:
        // Iterated Kalman Filter on Manifolds (FASTLIOv2)
        esekfom::esekf<state_ikfom, 12, input_ikfom> _iKFoM;
        std::mutex mtx_ikfom;

        State state, last_state;
        Extrinsics extr;
        SensorType sensor;
        IMUmeas last_imu;

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
        double imu_calib_time_;

        // Gravity
        double gravity_;

        // Flags
        bool imu_calibrated_;
        bool gravity_align_;
        bool calibrate_accel_;
        bool calibrate_gyro_;
        bool voxel_flag_;
        bool one_thread_;

        // Transformation matrices
        Eigen::Matrix4f T, T_prior;

        // OpenMP max threads
        int num_threads_;

        // IMU buffer
        boost::circular_buffer<IMUmeas> imu_buffer;
        std::mutex mtx_imu; // mutex for avoiding multiple thread access to the buffer
        std::condition_variable cv_imu_stamp;

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
            // Flags
        bool verbose_;  // whether to print out stats
        bool debug_;    // whether to copy original & deskewed pointclouds at each iteration

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

            // Other
        chrono::duration<double> elapsed_time;  // pointcloud callback elapsed time
        int deskew_size;                        // steps taken to deskew (FoV discretization)

    // FUNCTIONS

    public:
        Localizer();
        void init(bool one_thread=true);

        pcl::PointCloud<PointType>::Ptr get_pointcloud();
        pcl::PointCloud<PointType>::Ptr get_finalraw_pointcloud();

        pcl::PointCloud<PointType>::ConstPtr get_orig_pointcloud();
        pcl::PointCloud<PointType>::ConstPtr get_deskewed_pointcloud();
        pcl::PointCloud<PointType>::ConstPtr get_pc2match_pointcloud();

        State get_state();

        bool is_calibrated();

        void calculate_H(const state_ikfom&, const Matches&, Eigen::MatrixXd& H, Eigen::VectorXd& h);

        void propagateImu(const IMUmeas& imu);
        void propagateImu(double t1, double t2);

        void updateIMU(IMUmeas& raw_imu);
        void updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp);


    private:
        void init_iKFoM();
        void init_iKFoM_state();

        IMUmeas imu2baselink(IMUmeas& imu);

        pcl::PointCloud<PointType>::Ptr deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time);

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
            integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                         Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps);

        bool imuMeasFromTimeRange(double start_time, double end_time,
                                  boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
                                  boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it);

        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
            integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                 const std::vector<double>& sorted_timestamps,
                                 boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it,
                                 boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it);

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