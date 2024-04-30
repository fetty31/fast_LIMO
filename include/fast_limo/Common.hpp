#ifndef __FASTLIMO_COMMON_HPP__
#define __FASTLIMO_COMMON_HPP__

#ifndef HAS_CUPID
#include <cpuid.h>
#endif

#define FAST_LIMO_v "1.0.0"

// System
#include <ctime>
#include <iomanip>
#include <future>
#include <ios>
#include <sys/times.h>
#include <sys/vtimes.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

// FASTLIOv2
#include "use-ikfom.hpp"
#include "ikd_Tree.h"

// Boost
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace fast_limo {
  enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

    // MODULES
  class Localizer;
  class Mapper;

    // OBJECTS
  class State;
  class Plane;
  class Match;

    // STRUCTURES
  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity;
    union {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

  struct Extrinsics{
    struct SE3 {
        Eigen::Vector3f t;
        Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselin2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  };

  struct IMUmeas{
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  };

}

POINT_CLOUD_REGISTER_POINT_STRUCT(fast_limo::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (double, timestamp, timestamp))

typedef fast_limo::Point PointType;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<PointType>> MapPoints;

typedef std::vector<fast_limo::Match> Matches;
typedef std::vector<fast_limo::Plane> Planes;

#endif