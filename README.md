# Fast LIMO

A tightly coupled LiDAR-Inertial SLAM algorithm developed on top of [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) _C++_ libraries. This project's implementation is based on the existing algorithms [FASTLIO2](https://github.com/hku-mars/FAST_LIO), [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) and [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry).

`Fast-LIMO` has been developed as a thread-safe _C++_ library with [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) and [PCL](https://pointclouds.org/) as its only dependencies. This way, it can be used outside the ROS framework without any changes, making it more portable. This project acts as a ROS wrapper of the self-developed [fast_limo](include/fast_limo/)'s library.

Fast-LIMO stands for a multithreaded version of the approach _Localize Intensively Map Offline (LIMO)_ stated in [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) 's algorithm developed by [Huget57](https://github.com/Huguet57). 

## Disclaimer
If you plan to use fast LIMO the only thing I ask for is to give some love to the [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) project, which greatly influenced this work.

## References
This project relies upon [HKU-Mars](https://github.com/hku-mars)' open-source _C++_ libraries:
- Iterative Kalman Filters on Manifolds ([IKFoM](include/IKFoM/)) 
- Incremental KD-Tree ([ikd-Tree](include/ikd-Tree/)) 

## Dependencies
_C++14_ library ([fast_limo](include/fast_limo/)):
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PCL](https://pointclouds.org/) 1.8

ROS (Noetic) wrapper:
- [fast_limo](include/fast_limo/)
- [pcl_ros](http://wiki.ros.org/pcl_ros)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [tf2](https://wiki.ros.org/tf2)

## Quick Start
### 0. Cloning the repo
```sh
git clone https://github.com/fetty31/fast_LIMO
```

### 1. Build fast LIMO
Use default `catkin_make` or `catkin build` to build the code. By default it will compile under the `CMAKE_BUILD_TYPE="Release"` flag.

### 2. Run fast LIMO
```sh
roslaunch fast_limo fast_limo.launch
```

Afterwards, you should be seeing this output _(if `verbose` param is set to true)_:

to-do: add image of verbose output

## Configuration
Here the config file for fast LIMO will be explained.