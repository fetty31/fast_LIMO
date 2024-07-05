# Fast LIMO
<details>
    <summary>Table of Contents</summary>
    <ol>
        <li>
        <a href="#disclaimer">Disclaimer</a>
        </li>
        <li><a href="#dependencies">Dependencies</a>
        </li>
        <li>
        <a href="#quick-start">Quick Start</a>
        </li>
        <li>
        <a href="#configuration">Configuration</a>
        </li>
        <li>
        <a href="#approach">Approach</a>
        </li>
        <li>
        <a href="#references">References</a>
        </li>
    </ol>
</details>

<br>

A tightly coupled LiDAR-Inertial SLAM algorithm developed on top of [IKFoM](https://github.com/hku-mars/IKFoM) and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) _C++_ libraries. This project's implementation is based on the existing algorithms [FASTLIO2](https://github.com/hku-mars/FAST_LIO), [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) and [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry).

`Fast-LIMO` has been developed as a thread-safe _C++_ library with [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) and [PCL](https://pointclouds.org/) as its only dependencies. This way, it can be used outside the ROS framework without any changes, making it more portable. This project acts as a ROS wrapper of the self-developed [fast_limo](include/fast_limo/)'s library.

Fast-LIMO stands for a multithreaded version of the approach _Localize Intensively Map Offline (LIMO)_ stated in [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) 's algorithm developed by [Huget57](https://github.com/Huguet57). 

<div align="center">
<a> <img src="doc/xaloc.gif" alt="Logo" width="800"> </a>
<p align="center"><small>Formula Student trackdrive</small></p>
</div>
<br />

<div align="center">
<a> <img src="doc/kitti_0071.gif" alt="Logo" width="800"> </a>
<p align="center"><small>KITTI dataset</small></p>
</div>
<br />

## Disclaimer
If you plan to use fast LIMO please make sure to give some love to [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo), [FASTLIO2](https://github.com/hku-mars/FAST_LIO) and [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) projects, which greatly influenced this work.

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

### 1. Building fast LIMO
Use default `catkin_make` or `catkin build` to build the code. By default it will compile under the `CMAKE_BUILD_TYPE="Release"` flag.

### 2. Running fast LIMO
```sh
roslaunch fast_limo fast_limo.launch
```

Afterwards, you should be seeing this output _(if `verbose` param is set to true)_:

<div align="center">
<a> <img src="doc/verbose.png" alt="Logo" width="555"> </a>
<!-- <h3 align="center">Tailored MPC</h3> -->
</div>
<br />

## Configuration
Here the config file for fast LIMO will be explained.

## Approach
Here the main differences between fast LIMO and LIMO velo will be explained.

## References
This project relies upon [HKU-Mars](https://github.com/hku-mars)' open-source _C++_ libraries:
- Iterative Kalman Filters on Manifolds ([IKFoM](include/IKFoM/)) 
- Incremental KD-Tree ([ikd-Tree](include/ikd-Tree/)) 