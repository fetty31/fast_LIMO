#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

namespace fast_limo {
  
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
typedef pcl::PointCloud<PointType> PointCloudT;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> MapPoints;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> LocPoints;
typedef pcl::PointXYZINormal MatchPoint;
typedef pcl::PointCloud<MatchPoint> MatchPointCloud;