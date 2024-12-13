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

        fast_limo::Match::Match(const Eigen::Vector3f& p_global, 
                                const Eigen::Vector3f& p_local, 
                                const fast_limo::Plane& H) : p_global(p_global), p_local(p_local), plane(H)
        {
            this->dist = this->plane.dist2plane(p_global);
        }

        bool fast_limo::Match::lisanAlGaib(){
            return this->plane.good_fit();
        }

        void fast_limo::Match::update_global(fast_limo::State& s){
            Eigen::Vector4f p4_global = s.get_RT() * this->get_4Dlocal();
            this->p_global = p4_global.head(3);
        }

        Eigen::Vector4f fast_limo::Match::get_4Dglobal(){
            return Eigen::Vector4f(this->p_global(0), this->p_global(1), this->p_global(2), 1.0);
        }

        Eigen::Vector4f fast_limo::Match::get_4Dlocal(){
            return Eigen::Vector4f(this->p_local(0), this->p_local(1), this->p_local(2), 1.0);
        }

        Eigen::Vector3f fast_limo::Match::get_global_point(){
            return this->p_global;
        }

        Eigen::Vector3f fast_limo::Match::get_local_point(){
            return this->p_local;
        }
