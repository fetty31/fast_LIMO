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

#include "fast_limo/Objects/Match.hpp"

// class fast_limo::Match
    // public

        fast_limo::Match::Match(const Eigen::Vector3f& p, const fast_limo::Plane& H) : point(p), plane(H){
            this->dist = this->plane.dist2plane(p);
        }

        bool fast_limo::Match::lisanAlGaib(){
            return this->plane.good_fit();
        }

        Eigen::Vector4f fast_limo::Match::get_4Dpoint(){
            return Eigen::Vector4f(this->point(0), this->point(1), this->point(2), 1.0);
        }

        Eigen::Vector3f fast_limo::Match::get_point(){
            return this->point;
        }
