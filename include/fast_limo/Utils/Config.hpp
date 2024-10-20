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

#ifndef __FASTLIMO_CONFIG_HPP__
#define __FASTLIMO_CONFIG_HPP__

#include <ros/ros.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

namespace fast_limo {

  struct Config {

    struct Topics {
      string lidar;
      string imu;
    } topics;

    struct Extrinsics {
      Affine3f imu2baselink_T;
      Affine3f lidar2baselink_T;
    } extrinsics;

    struct Intrinsics {
      Vector3f accel_bias;
      Vector3f gyro_bias;
      Matrix3f imu_sm;
      /*(if your IMU doesn't comply with axis system ISO-8855, 
			this matrix is meant to map its current orientation with respect to the standard axis system)
				Y-pitch
				^   
				|  
				| 
				|
			Z-yaw o-----------> X-roll
			*/
    } intrinsics;

    struct Filters {
      Vector4f cropBoxMin;  // Crop filter
      Vector4f cropBoxMax;  // Crop filter
      bool crop_active;     // Crop filter
      Vector4f leafSize;    // Voxel grid filter
      bool voxel_active;    // Voxel grid filter
      double min_dist;      // Distance filter
      bool dist_active;     // Distance filter
      int rate_value;       // Rate filter
      bool rate_active;     // Rate filter
      float fov_angle;      // FoV filter
      bool fov_active;      // FoV filter
    } filters;

    struct iKFoM {
      struct Mapping {
        int NUM_MATCH_POINTS;   // Number of points that constitute a match
        int MAX_NUM_MATCHES;    // Max number of matches
        int MAX_NUM_PC2MATCH;   // Max number of points to match
        double MAX_DIST_PLANE;  // Max distance between points to be considered a plane
        double PLANE_THRESHOLD; // Plane threshold
        bool local_mapping;     // Local mapping flag
        struct iKDTree {
          float delete_param;
          float balance_param;
          float voxel_size;
          double cube_size;
          double rm_range;
        } ikdtree;
      } mapping;

      int MAX_NUM_ITERS;          // Max number of iterations
      vector<double> LIMITS;
      bool estimate_extrinsics;   // Estimate extrinsics flag
      double cov_gyro;            // Gyro covariance
      double cov_acc;             // Accelerometer covariance
      double cov_bias_gyro;       // Gyro bias covariance
      double cov_bias_acc;        // Accelerometer bias covariance
    } ikfom;

    // Flags
    bool gravity_align;    // Gravity alignment flag
    bool calibrate_accel;  // Accelerometer calibration flag
    bool calibrate_gyro;   // Gyro calibration flag
    bool time_offset;      // Time offset flag
    bool end_of_sweep;     // End of sweep flag

    bool debug;    // Debug flag
    bool verbose;  // Verbose flag

    // Other
    int sensor_type;        // LiDAR type
    int num_threads;        // Number of threads
    double imu_calib_time;  // IMU calibration time

    // Function to fill configuration using ROS NodeHandle
    void fill(ros::NodeHandle& nh) {
      // TOPICS
      nh.getParam("topics/input/lidar", topics.lidar);
      nh.getParam("topics/input/imu", topics.imu);

      // Other parameters
      nh.getParam("num_threads", num_threads);
      nh.getParam("sensor_type", sensor_type);

      nh.getParam("debug", debug);
      nh.getParam("verbose", verbose);

      nh.getParam("estimate_extrinsics", ikfom.estimate_extrinsics);
      nh.getParam("time_offset", time_offset);
      nh.getParam("end_of_sweep", end_of_sweep);

      nh.getParam("calibration/gravity_align", gravity_align);
      nh.getParam("calibration/accel", calibrate_accel);
      nh.getParam("calibration/gyro", calibrate_gyro);
      nh.getParam("calibration/time", imu_calib_time);

      // EXTRINSICS
      vector<float> tmp;

      nh.getParam("extrinsics/imu/t", tmp);
      if (tmp.size() >= 3)
        extrinsics.imu2baselink_T.translation() = Vector3f(tmp[0], tmp[1], tmp[2]);

      nh.getParam("extrinsics/imu/R", tmp);
      if (tmp.size() >= 9) {
        Matrix3f R_imu;
        R_imu << tmp[0], tmp[1], tmp[2],
                 tmp[3], tmp[4], tmp[5],
                 tmp[6], tmp[7], tmp[8];
        extrinsics.imu2baselink_T.linear() = R_imu;
      }

      nh.getParam("extrinsics/lidar/t", tmp);
      if (tmp.size() >= 3)
        extrinsics.lidar2baselink_T.translation() = Vector3f(tmp[0], tmp[1], tmp[2]);

      nh.getParam("extrinsics/lidar/R", tmp);
      if (tmp.size() >= 9) {
        Matrix3f R_lidar;
        R_lidar << tmp[0], tmp[1], tmp[2],
                   tmp[3], tmp[4], tmp[5],
                   tmp[6], tmp[7], tmp[8];

        extrinsics.lidar2baselink_T.linear() = R_lidar;
      }

      // INTRINSICS
      nh.getParam("intrinsics/accel/bias", tmp);
      if (tmp.size() >= 3)
        intrinsics.accel_bias = Vector3f(tmp[0], tmp[1], tmp[2]);

      nh.getParam("intrinsics/gyro/bias", tmp);
      if (tmp.size() >= 3)
        intrinsics.gyro_bias = Vector3f(tmp[0], tmp[1], tmp[2]);

      nh.getParam("intrinsics/accel/sm", tmp);
      if (tmp.size() >= 9) {
        intrinsics.imu_sm << tmp[0], tmp[1], tmp[2],
                             tmp[3], tmp[4], tmp[5],
                             tmp[6], tmp[7], tmp[8];
      }

      // FILTERS
      nh.getParam("filters/cropBox/active", filters.crop_active);
      nh.getParam("filters/cropBox/box/min", tmp);
      if (tmp.size() >= 4)
        filters.cropBoxMin = Vector4f(tmp[0], tmp[1], tmp[2], tmp[3]);

      nh.getParam("filters/cropBox/box/max", tmp);
      if (tmp.size() >= 4)
        filters.cropBoxMax = Vector4f(tmp[0], tmp[1], tmp[2], tmp[3]);

      nh.getParam("filters/voxelGrid/active", filters.voxel_active);
      nh.getParam("filters/voxelGrid/leafSize", tmp);
      if (tmp.size() >= 3)
        filters.leafSize = Vector4f(tmp[0], tmp[1], tmp[2], 1.);

      nh.getParam("filters/minDistance/active", filters.dist_active);
      nh.getParam("filters/minDistance/value", filters.min_dist);

      nh.getParam("filters/rateSampling/active", filters.rate_active);
      nh.getParam("filters/rateSampling/value", filters.rate_value);

      double fov_deg;
      nh.getParam("filters/FoV/active", filters.fov_active);
      nh.getParam("filters/FoV/value", fov_deg);
      filters.fov_angle = fov_deg * M_PI / 360.0; // Convert to radians, half FoV

      // iKFoM
      nh.getParam("iKFoM/Mapping/NUM_MATCH_POINTS", ikfom.mapping.NUM_MATCH_POINTS);
      nh.getParam("iKFoM/Mapping/MAX_NUM_MATCHES", ikfom.mapping.MAX_NUM_MATCHES);
      nh.getParam("iKFoM/Mapping/MAX_NUM_PC2MATCH", ikfom.mapping.MAX_NUM_PC2MATCH);
      nh.getParam("iKFoM/Mapping/MAX_DIST_PLANE", ikfom.mapping.MAX_DIST_PLANE);
      nh.getParam("iKFoM/Mapping/PLANES_THRESHOLD", ikfom.mapping.PLANE_THRESHOLD);
      nh.getParam("iKFoM/Mapping/LocalMapping", ikfom.mapping.local_mapping);

      nh.getParam("iKFoM/iKDTree/balance", ikfom.mapping.ikdtree.balance_param);
      nh.getParam("iKFoM/iKDTree/delete", ikfom.mapping.ikdtree.delete_param);
      nh.getParam("iKFoM/iKDTree/voxel", ikfom.mapping.ikdtree.voxel_size);
      nh.getParam("iKFoM/iKDTree/bb_size", ikfom.mapping.ikdtree.cube_size);
      nh.getParam("iKFoM/iKDTree/bb_range", ikfom.mapping.ikdtree.rm_range);

      nh.getParam("iKFoM/MAX_NUM_ITERS", ikfom.MAX_NUM_ITERS);
      nh.getParam("iKFoM/covariance/gyro", ikfom.cov_gyro);
      nh.getParam("iKFoM/covariance/accel", ikfom.cov_acc);
      nh.getParam("iKFoM/covariance/bias_gyro", ikfom.cov_bias_gyro);
      nh.getParam("iKFoM/covariance/bias_accel", ikfom.cov_bias_acc);

      double ikfom_limits;
      nh.getParam("iKFoM/LIMITS", ikfom_limits);
      ikfom.LIMITS = vector<double>(23, ikfom_limits);
    }

    static Config& getInstance() {
      static Config* config = new Config();
      return *config;
    }

   private:
    // Singleton pattern
    Config() = default;

    // Delete copy/move so extra instances can't be created/moved.
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;
  };

}

#endif // __FASTLIMO_CONFIG_HPP__
