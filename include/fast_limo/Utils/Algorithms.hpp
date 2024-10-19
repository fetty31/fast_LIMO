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
    }
    
}

#endif