#ifndef __FASTLIMO_MAPPER_HPP__
#define __FASTLIMO_MAPPER_HPP__

#include "fast_limo/Common.hpp"
#include "fast_limo/Objects/Match.hpp"
#include "fast_limo/Objects/State.hpp"
#include "fast_limo/Objects/Plane.hpp"

using namespace fast_limo;

class fast_limo::Mapper {

    // Variables

    private:
        KD_TREE<pcl::PointXYZ>::Ptr map;

        double last_map_time;

        int num_threads_;

    // Methods

    public:
        Mapper();

        void set_num_threads(int n);

        bool exists();
        int size();
        double last_time();

        Matches match(State, pcl::PointCloud<PointType>::ConstPtr&);

        void add(pcl::PointCloud<PointType>::ConstPtr&, double time, bool downsample=false);

    private:
        void build(pcl::PointCloud<PointType>::ConstPtr&);
        void add_pointcloud(pcl::PointCloud<PointType>::ConstPtr&, bool downsample=false);

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