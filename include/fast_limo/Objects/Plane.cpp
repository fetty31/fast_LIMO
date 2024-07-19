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

#include "fast_limo/Objects/Plane.hpp"

// class fast_limo::Plane
    // public

        fast_limo::Plane::Plane(const MapPoints& pts, const std::vector<float>& dts, 
                                Config::iKFoM::Mapping* config_ptr) : is_plane(false),
                                                                 cfg_ptr(config_ptr) { 
            if(not enough_points(pts)) return;
            if(not close_enough(dts)) return;

            // Get normal vector of plane between p points
            this->fit_plane(pts);
        }

        Eigen::Vector4f fast_limo::Plane::get_normal(){
            return this->n_ABCD;
        }

        bool fast_limo::Plane::good_fit(){
            return this->is_plane;
        }

        bool fast_limo::Plane::enough_points(const MapPoints& pts){
            return this->is_plane = pts.size() >= cfg_ptr->NUM_MATCH_POINTS;
        }

        bool fast_limo::Plane::close_enough(const std::vector<float>& dts){
            if(dts.size() < 1) return this->is_plane = false;
            return this->is_plane = dts.back() < cfg_ptr->MAX_DIST_PLANE;
        }

        float fast_limo::Plane::dist2plane(const Eigen::Vector3f& p) const {
            return n_ABCD(0) * p(0) + n_ABCD(1) * p(1) + n_ABCD(2) * p(2) + n_ABCD(3);
        }

        float fast_limo::Plane::dist2plane(const PointType& p) const {
            return n_ABCD(0) * p.x + n_ABCD(1) * p.y + n_ABCD(2) * p.z + n_ABCD(3);
        }

        bool fast_limo::Plane::on_plane(const Eigen::Vector3f& p) {
            if(not this->is_plane) return false;
            return std::fabs(this->dist2plane(p)) < cfg_ptr->PLANE_THRESHOLD;
        }

        bool fast_limo::Plane::on_plane(const PointType& p) {
            if(not this->is_plane) return false;
            return std::fabs(this->dist2plane(p)) < cfg_ptr->PLANE_THRESHOLD;
        }
    
    // private

        void fast_limo::Plane::fit_plane(const MapPoints& pts){
            // Estimate plane
            this->n_ABCD   = this->estimate_plane(pts);
            this->is_plane = this->plane_eval(n_ABCD, pts, cfg_ptr->PLANE_THRESHOLD);

            if(this->is_plane)
                this->centroid = this->get_centroid(pts);
            
        }

        Eigen::Vector4f fast_limo::Plane::estimate_plane(const MapPoints& pts){
            int NUM_MATCH_POINTS = pts.size();
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(NUM_MATCH_POINTS, 3);
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b(NUM_MATCH_POINTS, 1);
            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {
                A(j,0) = pts[j].x;
                A(j,1) = pts[j].y;
                A(j,2) = pts[j].z;
            }

            Eigen::Matrix<float, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
            Eigen::Vector4f pca_result;

            float n = normvec.norm();
            pca_result(0) = normvec(0) / n;
            pca_result(1) = normvec(1) / n;
            pca_result(2) = normvec(2) / n;
            pca_result(3) = 1.0 / n;

            return pca_result;
        }

        bool fast_limo::Plane::plane_eval(const Eigen::Vector4f& n, const MapPoints& pts, const float& thres){
            for (int j = 0; j < pts.size(); j++) {
                float res = n(0) * pts[j].x + n(1) * pts[j].y + n(2) * pts[j].z + n(3);
                if (fabs(res) > thres) return false;
            }

            return true;
        }

        Eigen::Vector3f fast_limo::Plane::get_centroid(const MapPoints& pts){
            int N = pts.size();
            Eigen::Vector3f centroid_vect;
            for (MapPoint p : pts) centroid_vect += p.getVector3fMap();
            return centroid_vect/static_cast<float>(N);
        }
