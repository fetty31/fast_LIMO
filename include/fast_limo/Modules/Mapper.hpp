#ifndef __FASTLIMO_MAPPER_HPP__
#define __FASTLIMO_MAPPER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Plane.hpp"
#include "fast_limo/Utils/Config.hpp"

using namespace fast_limo;

class fast_limo::Mapper {

    // Variables

    private:
        KD_TREE<MapPoint>::Ptr map;
        BoxPointType local_map_bb; // map's boundary box
        std::mutex mtx_local_map;

        Config::iKFoM::Mapping config;

        double last_map_time;

        int num_threads_;

        bool bb_init;

    // Methods

    public:
        Mapper();

        void set_num_threads(int n);
        void set_config(const Config::iKFoM::Mapping& cfg);

        bool exists();
        int size();
        double last_time();

        BoxPointType get_local_map();

        Matches match(State, pcl::PointCloud<PointType>::ConstPtr&);

        void add(pcl::PointCloud<PointType>::Ptr&, double time, bool downsample=false);
        void add(pcl::PointCloud<PointType>::Ptr&, State&, double time, bool downsample=false);

    private:
        void build(pcl::PointCloud<PointType>::Ptr&);

        void add_pointcloud(pcl::PointCloud<PointType>::Ptr&, bool downsample=false);

        void fov_segment(State&);
        
        void set_bb_dim(State&);

        Match match_plane(Eigen::Vector4f& p);

    // Singleton 

    public:
        static Mapper& getInstance() {
            static Mapper* mapper = new Mapper();
            return *mapper;
        }

    private:
        // Disable copy/move capabilities
        Mapper(const Mapper&) = delete;
        Mapper(Mapper&&) = delete;

        Mapper& operator=(const Mapper&) = delete;
        Mapper& operator=(Mapper&&) = delete;

};

#endif