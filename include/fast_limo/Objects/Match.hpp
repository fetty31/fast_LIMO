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

#ifndef __FASTLIMO_MATCH_HPP__
#define __FASTLIMO_MATCH_HPP__

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "fast_limo/Utils/PCL.hpp"
#include "fast_limo/Objects/Plane.hpp"


namespace fast_limo {
class Match{

    public:
        fast_limo::Plane plane;
        float dist;

        Match(const Eigen::Vector3f& p, const fast_limo::Plane& H);

        bool lisanAlGaib(); // whether is the chosen one :)

        Eigen::Vector4f get_4Dpoint();
        Eigen::Vector3f get_point();
    
    private:
        Eigen::Vector3f point;

};
    typedef std::vector<Match> Matches;
}

#endif