#ifndef __FASTLIMO_PLANE_HPP__
#define __FASTLIMO_PLANE_HPP__

#include "fast_limo/Common.hpp"

class fast_limo::Plane{

    public:
        PointType centroid;
        Eigen::Matrix<float, 4, 1> n_ABCD; // plane normal vector

        Plane();
        Plane(MapPoints& p, std::vector<float>& d);
        // float dist2plane(const PointType&) const;
        // bool on_plane(const PointType&);

    // private:
    //     void fill_plane(const MapPoints&);

};

#endif