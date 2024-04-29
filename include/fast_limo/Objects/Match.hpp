#ifndef __FASTLIMO_MATCH_HPP__
#define __FASTLIMO_MATCH_HPP__

#include "fast_limo/Common.hpp"

class fast_limo::Match{

    public:
        PointType point;
        fast_limo::Plane plane;
        float dist;

        Match(const PointType& p, const fast_limo::Plane& H);

        bool lisanAlGaib(); // whether is the chosen one :)

};

#endif