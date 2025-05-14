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

#include "fast_limo/Common.hpp"
#include "fast_limo/Objects/Octree.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Plane.hpp"
#include "fast_limo/Utils/Config.hpp"

using namespace fast_limo;

class fast_limo::Mapper {

    // Variables

    private:
        octree::Octree octree_;

        Config::iKFoM::Mapping config;

        double last_map_time;

        int num_threads_;

    public:
        Matches matches;

    // Methods

    public:
        Mapper();

        void set_num_threads(int n);
        void set_config(const Config::iKFoM::Mapping& cfg);

        bool exists();
        int size();
        double last_time();

        Matches match(State, pcl::PointCloud<PointType>::Ptr&);

        void add(pcl::PointCloud<PointType>::Ptr&, double time);

    private:
        Match match_plane(Eigen::Vector4f& p, Eigen::Vector4f& p_local);

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

#endif
