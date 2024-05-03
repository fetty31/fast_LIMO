#ifndef __FASTLIMO_MATCH_HPP__
#define __FASTLIMO_MATCH_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Objects/Plane.hpp"

class fast_limo::Match{

    public:
        fast_limo::Plane plane;
        float dist;

        Match(const Eigen::Vector3f& p, const fast_limo::Plane& H);

        bool lisanAlGaib(); // whether is the chosen one :)

        Eigen::Vector4f get_point();
    
    private:
        Eigen::Vector3f point;

};

#endif