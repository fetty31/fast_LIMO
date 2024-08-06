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

#ifndef __FASTLIMO_LOOPER_HPP__
#define __FASTLIMO_LOOPER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Utils/Config.hpp"
#include "fast_limo/Utils/GNSStf.hpp"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace fast_limo;

class fast_limo::Looper {

    // VARIABLES

    private:
            // GTSAM
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init_estimates;
        gtsam::ISAM2* iSAM_;
        gtsam::Pose3 out_estimate;

        bool priorAdded;
        uint64_t global_idx;

        std::mutex graph_mtx;

        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
        gtsam::noiseModel::Diagonal::shared_ptr gnss_noise;
        // gtsam::noiseModel::Base::shared_ptr gnss_noise;

            // Keyframes
        boost::circular_buffer<std::pair<State, pcl::PointCloud<PointType>::Ptr>> keyframes;
        std::mutex kf_mtx;

            // GNSS transformer
        GNSStf gnss_tf;

            // Aux var.
        bool initFlag;

    // FUNCTIONS

    public:
        Looper();
        ~Looper();

        void init(); // To DO: add config struct as input

        State get_state();
        void get_state(State& s);

        void solve();

        void update(State s, pcl::PointCloud<PointType>::Ptr&);
        void update(State s);
        void update(double lat, double lng, double alt);

    private:
        void update(gtsam::Pose3 pose);
        gtsam::Pose3 fromLIMOtoGTSAM(const State& s);

    // SINGLETON 

    public:
        static Looper& getInstance(){
            static Looper* loop = new Looper();
            return *loop;
        }

    private:
        // Disable copy/move functionality
        Looper(const Looper&) = delete;
        Looper& operator=(const Looper&) = delete;
        Looper(Looper&&) = delete;
        Looper& operator=(Looper&&) = delete;

};

#endif