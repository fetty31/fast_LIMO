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

#ifndef _FASTLIMO_LOOP_CONFIG_HPP_
#define _FASTLIMO_LOOP_CONFIG_HPP_

#include "fast_limo/Common.hpp"

struct fast_limo::LoopConfig{

    struct Topics{
        std::string gnss_fix;
    } topics;

    struct ScanContext{
        int NUM_EXCLUDE_RECENT;
        int NUM_CANDIDATES_FROM_TREE;
        int PC_NUM_RING;
        int PC_NUM_SECTOR;
        float PC_MAX_RADIUS;
        float SC_THRESHOLD;
        float SEARCH_RATIO;
    } scancontext;

    struct PoseGraph{
        std::vector<double> prior_cov;
        std::vector<double> odom_cov;
        std::vector<double> gnss_cov;
        std::vector<double> loop_cov;
        int min_num_states;
    } posegraph;

    struct RadiusSearch{
        float RADIUS;
        bool active;
    } radiussearch;

    struct ICP{
        float MAX_DIST;
        float TF_EPSILON;
        float EUC_FIT_EPSILON;
        float FIT_SCORE;
        int MAX_ITERS;
        int RANSAC_ITERS;
        int WINDOW_SIZE;
    } icp;

    struct KeyFrames{
        std::pair<float, float> odom_diff;
        float gnss_diff;
    } kf;

};

#endif