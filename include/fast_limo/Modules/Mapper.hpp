#ifndef __FASTLIMO_MAPPER_HPP__
#define __FASTLIMO_MAPPER_HPP__

#include "fast_limo/Common.hpp"

using namespace fast_limo;

class fast_limo::Mapper {

    // Variables

    private:
        KD_TREE<pcl::PointXYZ>::Ptr map;

        double last_map_time;

        int num_threads_;

    // Methods

    public:
        Mapper(int num_threads);

        bool is_built();
        int size();
        double last_time()

        Matches match(const State&, pcl::PointCloud<PointType>::ConstPtr&);

        void add(pcl::PointCloud<PointType>::ConstPtr&, double time, bool downsample=false);

    private:
        void build(pcl::PointCloud<PointType>::ConstPtr&);
        void add_pointcloud(pcl::PointCloud<PointType>::ConstPtr&, bool downsample=false);

        Match match_plane(const PointType& p)

    // Singleton 

    public:
        Mapper& getInstance() {
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