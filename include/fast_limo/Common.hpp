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

#ifndef __FASTLIMO_COMMON_HPP__
#define __FASTLIMO_COMMON_HPP__

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#define FAST_LIMO_v "2.1.0"

// System
#include <ctime>
#include <iomanip>
#include <future>
#include <ios>
#include <sys/times.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <string>

#include <climits>
#include <cmath>

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

#include <memory>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

// FASTLIOv2
#include "IKFoM/use-ikfom.hpp"

// Boost
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>
#include <boost/range/adaptor/filtered.hpp>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

namespace fast_limo {
  enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

    // MODULES
  class Localizer;
  class Mapper;

    // OBJECTS
  class State;
  class Plane;
  class Match;
  namespace octree {
    struct Octree;
  }

    // UTILS
  struct Config;

    // STRUCTURES
  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}
    Point(float x, float y, float z): data{x, y, z, 1.f} {}

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
    SE3 imu2baselink;
    SE3 lidar2baselink;
    Eigen::Matrix4f imu2baselink_T;
    Eigen::Matrix4f lidar2baselink_T;
  };

  struct IMUmeas{
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
    Eigen::Quaternionf q;
  };

#if PCL_VERSION_COMPARE(<, 1, 11, 0)
	// Use Boost for older versions of PCL
	template <typename T>
	using shared_ptr = boost::shared_ptr<T>;

	template <typename T, typename... Args>
	boost::shared_ptr<T> make_shared(Args&&... args) {
    	return boost::make_shared<T>(std::forward<Args>(args)...);
	}
#else
	// Use std::shared_ptr for PCL >= 1.10.0
	template <typename T>
	using shared_ptr = std::shared_ptr<T>;

	template <typename T, typename... Args>
	std::shared_ptr<T> make_shared(Args&&... args) {
    	return std::make_shared<T>(std::forward<Args>(args)...);
	}
#endif

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
typedef pcl::PointXYZ MapPoint;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> MapPoints;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> LocPoints;

typedef std::vector<fast_limo::Match> Matches;
typedef std::vector<fast_limo::Plane> Planes;
typedef std::vector<fast_limo::State> States;

#endif
