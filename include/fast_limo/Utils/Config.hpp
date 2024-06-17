#ifndef _FAST_LIMO_CONFIG_HPP_
#define _FAST_LIMO_CONFIG_HPP_

#include "fast_limo/Common.hpp"

struct fast_limo::Config{

    struct Topics{
        std::string lidar;
        std::string imu;
    } topics;

    struct Extrinsics{
        std::vector<float> baselink2imu_t;
        std::vector<float> baselink2imu_R;
        std::vector<float> baselink2lidar_t;
        std::vector<float> baselink2lidar_R;
    } extrinsics;

    struct Intrinsics{
        std::vector<float> accel_bias;
        std::vector<float> gyro_bias;
        std::vector<float> imu_sm;
    } intrinsics;

    struct Filters{
        std::vector<float> cropBoxMin;  // crop filter
        std::vector<float> cropBoxMax;  // crop filter
        bool crop_active;               // crop filter
        std::vector<float> leafSize;    // voxel grid filter
        bool voxel_active;              // voxel grid filter
        double min_dist;                // norm/dist filter
        bool dist_active;               // norm/dist filter
        int rate_value;                 // time rate filter
        bool rate_active;               // time rate filter
    } filters;

    struct iKFoM{
        int MAX_NUM_ITERS;
        int NUM_MATCH_POINTS;
        double MAX_DIST_PLANE;
        double PLANE_THRESHOLD;
        std::vector<double> LIMITS;
        bool estimate_extrinsics;
        double cov_gyro;
        double cov_acc;
        double cov_bias_gyro;
        double cov_bias_acc;
    } ikfom;

    // Flags
    bool gravity_align;
    bool calibrate_accel;
    bool calibrate_gyro;
    bool time_offset;

    bool debug;
    bool verbose;

    // Other
    int sensor_type;
    int num_threads;
    double imu_calib_time;
    double time_delay;

};

#endif