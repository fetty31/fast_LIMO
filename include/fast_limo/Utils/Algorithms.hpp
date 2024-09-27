#ifndef _FAST_LIMO_ALGORITHMS_HPP_
#define _FAST_LIMO_ALGORITHMS_HPP_

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
