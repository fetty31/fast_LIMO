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

#ifndef __FASTLIMO_ALGORITHMS_HPP__
#define __FASTLIMO_ALGORITHMS_HPP__

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

#include "fast_limo/Utils/PCL.hpp"
#include <Eigen/Dense>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

namespace fast_limo {
    namespace algorithms {

        template <typename Array>
        int binary_search_tailored(const Array& sorted_v, double t) {
            int high, mid, low;
            low = 0; high = sorted_v.size()-1;
            
            while(high >= low){
                mid = (low + high)/2;
                (sorted_v[mid].time > t) ? high = mid - 1 : low = mid + 1;
            }

            // Return the leftest value (older time stamp)
            if(high < 0) return 0;
            return high;
        }

        bool estimate_plane(Eigen::Vector4f& pabcd, const MapPoints& pts, double& thresh){
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
            
            pabcd = pca_result;

            for (int j = 0; j < pts.size(); j++) {
                double dist2point = std::fabs(pabcd(0) * pts[j].x 
                                            + pabcd(1) * pts[j].y
                                            + pabcd(2) * pts[j].z
                                            + pabcd(3));
                
                if (dist2point > thresh)
                    return false;
            }

            return true;
        
        }

    }
}

#endif