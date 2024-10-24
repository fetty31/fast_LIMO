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

#include "fast_limo/Common.hpp"

namespace fast_limo::algorithms {

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

#endif