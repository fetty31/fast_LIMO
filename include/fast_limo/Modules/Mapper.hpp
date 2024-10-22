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

#include "octree2/Octree.h"
#include "ikd-Tree/ikd_Tree/ikd_Tree.h"

#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Utils/Config.hpp"

using namespace fast_limo;

namespace fast_limo {
  class IKDTree {

    public:
      KD_TREE<MapPoint>::Ptr map;

      Config::iKFoM::Mapping config;

      double last_map_time_;

      int num_threads_;;

      IKDTree();

      bool exists();
      int size();
      double last_time();

      void knn(const MapPoint& p,
               int& k,
               MapPoints& near_points,
               std::vector<float>& sqDist);

      void add(PointCloudT::Ptr&, double time, bool downsample=true);

    public:
      void build(PointCloudT::Ptr&);


    public:
      static IKDTree& getInstance() {
        static IKDTree* ikd = new IKDTree();
        return *ikd;
      }

    private:
      // Disable copy/move capabilities
      IKDTree(const IKDTree&) = delete;
      IKDTree(IKDTree&&) = delete;

      IKDTree& operator=(const IKDTree&) = delete;
      IKDTree& operator=(IKDTree&&) = delete;
  };

  class Octree {

    public:
      thuni::Octree map;

      Config::iKFoM::Mapping config;

      double last_map_time_;

      int num_threads_;;

      Octree();

      bool exists();
      int size();
      double last_time();

      void knn(const MapPoint& p,
               int& k,
               MapPoints& near_points,
               std::vector<float>& sqDist);

      void add(PointCloudT::Ptr&, double time, bool downsample=true);

    public:
      void build(PointCloudT::Ptr&);

    public:
      static Octree& getInstance() {
        static Octree* ikd = new Octree();
        return *ikd;
      }

    private:
      Octree(const Octree&) = delete;
      Octree(Octree&&) = delete;

      Octree& operator=(const Octree&) = delete;
      Octree& operator=(Octree&&) = delete;
  };


typedef IKDTree Mapper;

}

#endif