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

`Fast-LIMO` stands for a multithreaded version of the approach _Localize Intensively Map Offline (LIMO)_ stated in [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo) 's algorithm developed by [Huget57](https://github.com/Huguet57). 

<div align="center">
<a> <img src="doc/xaloc.gif" alt="Logo" width="800"> </a>
<a href="https://youtu.be/ly_ax8w-T7E?si=sDFiMFtRN5jRwWKC" align="center"><small>Formula Student race car XALOC</small></a>
</div>
<br />

<div align="center">
<a> <img src="doc/kitti_0071.gif" alt="Logo" width="800"> </a>
<a href="https://www.cvlibs.net/datasets/kitti/index.php" align="center"><small>KITTI dataset</small></a>
</div>
<br />

<div align="center">
<a> <img src="doc/cat15.gif" alt="Logo" width="800"> </a>
<a href="https://youtu.be/mk9U0lRWr-0?si=j4mM6e5dzihfCLJM" align="center"><small>Formula Student race car CAT15X</small></a>
</div>
<br />

## Disclaimer
If you plan to use `Fast-LIMO` please make sure to give some love to [LIMO-Velo](https://github.com/Huguet57/LIMO-Velo), [FASTLIO2](https://github.com/hku-mars/FAST_LIO) and [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) projects, which greatly influenced this work.

## Dependencies
<details open>
    <summary>C++14 Fast-LIMO library :</summary>
    <ol>
        <li>
        <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen3</a>
        </li>
        <li>
        <a href="https://pointclouds.org/">PCL (1.8)</a>
        </li>
    </ol>
</details>

<details>
    <summary>ROS (Noetic) wrapper:</summary>
    <ol>
        <li>
        <a href="./include/fast_limo/">fast_limo</a>
        </li>
        <li><a href="http://wiki.ros.org/pcl_ros">pcl_ros</a>
        </li>
        <li>
        <a href="http://wiki.ros.org/sensor_msgs">sensor_msgs</a>
        </li>
        <li>
        <a href="http://wiki.ros.org/geometry_msgs">geometry_msgs</a>
        </li>
        <li>
        <a href="https://wiki.ros.org/tf2">tf2</a>
        </li>
    </ol>
</details>


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
</div>
<br />

You can also run `Fast-LIMO` together with an rviz instance with:
```sh
roslaunch fast_limo fast_limo rviz:=true
```

## Configuration
Here the config file for fast LIMO will be explained.

## Approach
Here the main differences between fast LIMO and LIMO velo will be explained.

## References
This project relies upon [HKU-Mars](https://github.com/hku-mars)' open-source _C++_ libraries:
- Iterative Kalman Filters on Manifolds ([IKFoM](include/IKFoM/)) 
- Incremental KD-Tree ([ikd-Tree](include/ikd-Tree/)) 