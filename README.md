# Fast-LIMO Relocation Branch 🚀
<details>
    <summary>Table of Contents</summary>
    <ol>
        <li>
        <a href="#introduction">Introduction</a>
        </li>
        <li><a href="#conecptyal-overview">Conceptual Overview</a>
        </li>
        <li><a href="#installation">Installation</a>
        </li>
        <li>
        <a href="#configuration-parameters">Configuration Parameters</a>
        </li>
        <li>
        <a href="#getting-started">Getting Started</a>
        </li>
    </ol>
</details>

## Introduction 

This branch adds a map-based relocalization module to the Fast-LIMO LiDAR-Inertial SLAM pipeline. By running two nodes in parallel—one for SLAM and one for relocalization—you can recover the robot’s pose in a previously built map (`.pcd`). Once the relocalizer finds a valid pose, it sends the fully transformed map to the SLAM node, which then loads the static map and stops incremental mapping.
---

## Conceptual Overview 🤖

* **fast\_limo (SLAM pipeline)**

  * Incrementally builds a LiDAR+IMU map
  * Exposes `/fast_limo/save_map` service to dump the current map as a `.pcd`

* **fast\_limo\_reloca (Relocator)**

  * Accumulates LiDAR scans until a travel distance threshold is exceeded
  * **Modes**:

    * **Global** (`mode=false`): search the entire map
    * **Local** (`mode=true`): restrict search around `/initialpose` from RViz
  * **Workflow**:

    1. `updateCloud()` ← accumulate scans
    2. **KISS-Matcher** for coarse pose estimation
    3. **Nano-GICP** for pose refinement (fitness score)
    4. On success:

       * Transform full map → call `/fast_limo/send_pointcloud` service → publish map on `/fast_limo_reloca/full_map` topic

* **Node Interaction** 🔄

  ```bash
  # Terminal A: run SLAM
  roslaunch fast_limo fast_limo.launch

  # Terminal B: run Relocalizer
  roslaunch fast_limo reloca.launch rviz:=true map_name:=campus_nord
  ```

  When a valid pose is found, `fast_limo_reloca` calls `/fast_limo/send_pointcloud` (in `world` frame) and SLAM switches to map-based localization. 
  
  _Note that the received map will be in `world` frame (instead of `map` frame) so there won't be any localization discontinuity._

---

## Installation 🛠️

### 1. Clone Repo

```bash
git clone https://github.com/fetty31/fast_LIMO.git
cd fast_LIMO
git checkout relocation/KISS-matcher
```

### 2. Install ROBIN, KISS-Matcher & Nano-GICP

1. **ROBIN**
   Follow the official instructions on GitHub:
   [https://github.com/MIT-SPARK/ROBIN](https://github.com/MIT-SPARK/ROBIN)

2. **KISS-Matcher**

   ```bash
   git clone https://github.com/fetty31/KISS-Matcher.git
   cd KISS-Matcher
   make cppinstall_matcher_only
   ```

3. **Nano-GICP**

   ```bash
   # In the same workspace where you build fast_limo:
   git clone https://github.com/fetty31/nano_gicp.git
   ```

### 4. Build Fast-LIMO

* **ROS Noetic (catkin)**

  ```bash
  catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

---

## Configuration Parameters 🔧

All relocalizer parameters live in `RelocaConfig` (set via `reloca.launch`):

| Parameter            | Type   | Default | Description                                                              |
| -------------------- | ------ | ------- | ------------------------------------------------------------------------ |
| `mode`               | bool   | `true`  | `true` = local (requires an `/initialpose` in RViz) <br>`false` = global |
| `map_path`           | string | —       | Filesystem path to the `.pcd` map                                        |
| `distance_threshold` | double | `10.0`  | Travel distance (in meters) before triggering relocalization (only valid if `mode`==true)             |
| `inliers_threshold`  | int    | `10`    | Minimum KISS-Matcher inliers to accept a coarse solution                 |
| `score`              | double | `100.0` | Maximum Nano-GICP fitness score to accept the refined solution           |

---

## Getting Started 🚀

Saving a map while doing SLAM:
1. **Launch SLAM**

   ```bash
   roslaunch fast_limo fast_limo.launch
   ```

2. **Save a map manually**

   ```bash
   rosservice call /fast_limo/save_map \
     "{ full_path: { data: '/path/fast_LIMO/maps/test.pcd' } }"
   ```

   Use that `.pcd` via `map_name`.

Relocalizing in a previously saved map:
1. **Launch SLAM**

   ```bash
   roslaunch fast_limo fast_limo.launch
   ```

2. **Launch Relocalization**

   ```bash
   roslaunch fast_limo reloca.launch map_name:=test
   ```

Happy relocalizing! 😊
