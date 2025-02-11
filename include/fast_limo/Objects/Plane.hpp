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

#ifndef __FASTLIMO_PLANE_HPP__
#define __FASTLIMO_PLANE_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Utils/Config.hpp"

class fast_limo::Plane{

    public:

        Plane(const MapPoints& p, const std::vector<float>& d, 
              Config::iKFoM::Mapping* config_ptr);
        Plane() = default;

        Eigen::Vector4f get_normal();
        bool good_fit();

        float dist2plane(const Eigen::Vector3f&) const;
        float dist2plane(const PointType&) const;

        bool on_plane(const Eigen::Vector3f&);
        bool on_plane(const PointType&);
        
        bool enough_points(const MapPoints& p);
        bool close_enough(const std::vector<float>& d);

    private:
        Eigen::Vector3f centroid;
        Eigen::Vector4f n_ABCD; // plane normal vector
        bool is_plane;

        Config::iKFoM::Mapping* cfg_ptr;

        void fit_plane(const MapPoints&);

        Eigen::Vector4f estimate_plane(const MapPoints&);

        bool plane_eval(const Eigen::Vector4f&, const MapPoints&, const float&);

        Eigen::Vector3f get_centroid(const MapPoints&);

};

#endif
