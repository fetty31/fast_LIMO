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

 #ifndef __FASTLIMO_RELOCATOR_HPP__
 #define __FASTLIMO_RELOCATOR_HPP__
 
 #include "fast_limo/Common.hpp"
 #include "fast_limo/Utils/Config.hpp"
 
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/registration/icp.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/voxel_grid.h>
 #include <nav_msgs/Odometry.h>
 
 #include <kiss_matcher/FasterPFH.hpp>
 #include <kiss_matcher/GncSolver.hpp>
 #include <kiss_matcher/KISSMatcher.hpp>
 
 #include <nano_gicp/point_type_nano_gicp.hpp> //changed from pcl::PointXYZI to PointTypeNano in this headerfile
 #include <nano_gicp/nano_gicp.hpp>
 
 #include <cmath>
 #include <limits>
 
 using namespace fast_limo;
 
 class fast_limo::Relocator {
 public:
 
    Relocator();
    void init(const RelocaConfig& cfg);
    void updateCloud(pcl::PointCloud<PointType>::Ptr& pc);
    void updateState(const nav_msgs::Odometry::ConstPtr& msg);
    void updateInitialPose(std::vector<double> init_state);

    inline Eigen::Vector3f get_pose() { return this->p; }
    inline Eigen::Quaternionf get_orientation() { return this->q; }

    void get_full_map(pcl::PointCloud<PointType>::Ptr& full_map){
        full_map = full_map_ds;
    }
    void get_full_map_transformed(pcl::PointCloud<PointType>::Ptr& full_map){
        full_map = this->full_map_transformed_;
    }
    bool is_relocated(){
        return relocated;
    }

 private:
 
    pcl::PointCloud<PointType>::Ptr target_map_, source_cloud_;
    pcl::PointCloud<PointType>::Ptr aligned_cloud_, aligned_cloud_gicp;
    pcl::PointCloud<PointType>::Ptr full_map_, full_map_ds, full_map_transformed_;
    
    Eigen::Vector3f p;
    Eigen::Quaternionf q;

    double distance_traveled = 0;
    double last_x = std::nan(""), last_y;
    bool relocated = false, recived_estimated_pose = false;
    std::array<double,3> init_state_{ 0.0, 0.0, 0.0};

    Eigen::Matrix4f kiss_transformation_;

    RelocaConfig cfg_;

    nano_gicp::NanoGICP<PointTypeNano, PointTypeNano> m_nano_gicp;

    bool relocation();
    void passThroughFilter(pcl::PointCloud<PointType>::Ptr& cloud, float size);
    void voxelGridFilter(pcl::PointCloud<PointType>::Ptr& cloud, float voxel_size);
    bool applyGICP();
    bool applyKissMatcher();
    void load_map();
    void transformFullMap();
    bool enough_distance_traveled();
    void reset();
    
    // SINGLETON 

    public:
        static Relocator& getInstance(){
            static Relocator* reloca = new Relocator();
            return *reloca;
        }

    private:
        // Disable copy/move functionality
        Relocator(const Relocator&) = delete;
        Relocator& operator=(const Relocator&) = delete;
        Relocator(Relocator&&) = delete;
        Relocator& operator=(Relocator&&) = delete;

 
 };
 
 #endif // __FASTLIMO_RELOCATOR_HPP__