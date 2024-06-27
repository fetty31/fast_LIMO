# Fast LIMO

A tightly coupled LiDAR-Inertial SLAM algorithm developed on top of [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) _C++_ libraries. This project's implementation is based on the existing algorithms [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) and [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry).

`Fast-LIMO` has been developed as a thread-safe _C++_ library with [IKFoM](include/IKFoM/) and [ikd-Tree](include/ikd-Tree/) as its only dependencies. This way, it can be used outside the ROS framework without any changes, making it more portable. This project acts as a ROS wrapper of the self-developed [fast_limo](include/fast_limo/)'s library using it in a _multi-threaded_ fashion in [main.cpp](src/main.cpp) and in a standard _one-threaded_ fashion in [onethread.cpp](src/onethread.cpp).

Fast-LIMO stands for an improved/modified version of the approach _Localize Intensively Map Offline (LIMO)_ stated in LIMO-Velo's algorithm. 

## Dependencies
_C++14_ library ([fast_limo](include/fast_limo/)):
- Iterative Kalman Filters on Manifolds ([IKFoM](include/IKFoM/)) _C++_ library
- Incremental KD Trees ([ikd-Tree](include/ikd-Tree/)) _C++_ library
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PCL](https://pointclouds.org/)

ROS (Noetic) wrapper:
- [fast_limo](include/fast_limo/)
- [pcl_ros](http://wiki.ros.org/pcl_ros)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [tf2](https://wiki.ros.org/tf2)

## Quick Start
