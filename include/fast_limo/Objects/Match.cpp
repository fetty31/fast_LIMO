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
