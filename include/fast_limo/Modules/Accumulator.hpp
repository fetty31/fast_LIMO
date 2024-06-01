#ifndef __FASTLIMO_LOCALIZER_HPP__
#define __FASTLIMO_LOCALIZER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Modules/Mapper.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/Plane.hpp"

using namespace fast_limo;

class Accumulator {

    // VARIABLES
    std::deque<PointType> lidar_buffer;

    // FUNCTIONS

    // SINGLETON

    public:
        static Accumulator& getInstance(){
            static Accumulator* accum = new Accumulator();
            return *accum;
        }

    private:
        // Disable copy/move functionality
        Accumulator(const Accumulator&) = delete;
        Accumulator& operator=(const Accumulator&) = delete;
        Accumulator(Accumulator&&) = delete;
        Accumulator& operator=(Accumulator&&) = delete;

};