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

#ifndef __FASTLIMO_MAPPER_HPP__
#define __FASTLIMO_MAPPER_HPP__

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
#include <chrono>
#include <string>

#include <climits>
#include <cmath>

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

#include "ikd-Tree/ikd_Tree/ikd_Tree.h"

#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Utils/Config.hpp"

using namespace fast_limo;

namespace fast_limo {
class Mapper {

    // Variables

    public:
        KD_TREE<MapPoint>::Ptr map;
        BoxPointType local_map_bb; // map's boundary box
        std::mutex mtx_local_map;

        Config::iKFoM::Mapping config;

        double last_map_time;

        int num_threads_;

        bool bb_init;

    // Methods

    public:
        Mapper();

        void set_num_threads(int n);
        void set_config(const Config::iKFoM::Mapping& cfg);

        bool exists();
        int size();
        double last_time();

        BoxPointType get_local_map();

        // Matches match(State, pcl::PointCloud<PointType>::ConstPtr&);

        void add(pcl::PointCloud<PointType>::Ptr&, double time, bool downsample=false);
        void add(pcl::PointCloud<PointType>::Ptr&, State&, double time, bool downsample=false);
        
        // Match match_plane(Eigen::Vector4f& p);

    public:
        void build(pcl::PointCloud<PointType>::Ptr&);

        void add_pointcloud(pcl::PointCloud<PointType>::Ptr&, bool downsample=false);

        void fov_segment(State&);
        
        void set_bb_dim(State&);


    // Singleton 

    public:
        static Mapper& getInstance() {
            static Mapper* mapper = new Mapper();
            return *mapper;
        }

    private:
        // Disable copy/move capabilities
        Mapper(const Mapper&) = delete;
        Mapper(Mapper&&) = delete;

        Mapper& operator=(const Mapper&) = delete;
        Mapper& operator=(Mapper&&) = delete;

};

}

#endif