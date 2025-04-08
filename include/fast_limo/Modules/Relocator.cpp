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


#include "fast_limo/Modules/Relocator.hpp"

// class fast_limo::Relocator
// public

std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<fast_limo::Point>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
    vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
}

Relocator::Relocator() {
    target_map_.reset(new pcl::PointCloud<PointType>);
    source_cloud_.reset(new pcl::PointCloud<PointType>);
    aligned_cloud_.reset(new pcl::PointCloud<PointType>);
    full_map_.reset(new pcl::PointCloud<PointType>);
    full_map_transformed_.reset(new pcl::PointCloud<PointType>);
    full_map_ds.reset(new pcl::PointCloud<PointType>);
    aligned_cloud_gicp.reset(new pcl::PointCloud<PointType>);

    p = Eigen::Vector3f::Zero();
    q = Eigen::Quaternionf::Identity();


    // Initialize NanoGICP with default parameters
    m_nano_gicp = nano_gicp::NanoGICP<PointTypeNano, PointTypeNano>();
    m_nano_gicp.setMaxCorrespondenceDistance(2.0);
    m_nano_gicp.setNumThreads(0);
    m_nano_gicp.setCorrespondenceRandomness(15);
    m_nano_gicp.setMaximumIterations(32);
    m_nano_gicp.setTransformationEpsilon(0.01);
    m_nano_gicp.setEuclideanFitnessEpsilon(0.01);
    m_nano_gicp.setRANSACIterations(5);
    m_nano_gicp.setRANSACOutlierRejectionThreshold(1.0);

}

void Relocator::init(const RelocaConfig& cfg) {
    this->cfg_ = cfg; // Load configuration parameters
    this->load_map(); // Import PCD map
}

bool Relocator::enough_distance_traveled() {
    return this->distance_traveled > this->cfg_.distance_threshold;
}

void Relocator::updateInitialPose(std::vector<float> init_state){
    this->init_state_ = init_state;
    this->recived_estimated_pose = true; // Todo turn it back to false once tried
}

void Relocator::updateCloud(pcl::PointCloud<PointType>::Ptr& pc) {

    // Accumulate the point cloud
    *this->source_cloud_ += *pc;

    bool should_relocate = (!cfg_.mode && enough_distance_traveled()) ||
                           (cfg_.mode && recived_estimated_pose);

    if(should_relocate){
        std::cout << "Distance traveled: " << this->distance_traveled << " meters" << std::endl;
        std::cout << "Reloca Corriendo..." << std::endl;
        pcl::copyPointCloud(*this->full_map_, *this->target_map_);
        this->relocated = this->relocation();
        if(this->relocated) this->transformFullMap();
        else this->source_cloud_->clear();
    }

}

void Relocator::updateState(const nav_msgs::Odometry::ConstPtr& msg) {

    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    
    static double last_x = current_x;
    static double last_y = current_y;
    
    double dx = current_x - last_x;
    double dy = current_y - last_y;
    double delta_distance = std::sqrt(dx * dx + dy * dy);
    
    if(delta_distance > 0.1){
        this->distance_traveled += delta_distance;
        last_x = current_x;
        last_y = current_y;    
    }  
}

// private
void Relocator::load_map() {

    std::cout << "Starting loading map from " << this->cfg_.map_path << " ..." << std::endl;
    if(pcl::io::loadPCDFile<PointType>(this->cfg_.map_path, *this->full_map_) == 0){
        pcl::copyPointCloud(*this->full_map_, *this->full_map_ds);
        this->voxelGridFilter(this->full_map_ds, 0.6f);
        std::cout << "Map loaded successfully" << std::endl;
    } else {
        std::cout << "Failed to load map" << std::endl;
    }

}

void Relocator::passThroughFilter(pcl::PointCloud<PointType>::Ptr& cloud, float size)
{
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-size+init_state_[0], size+init_state_[0]);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-size+init_state_[1], size+init_state_[1]);
    pass.filter(*cloud);

}

void Relocator::voxelGridFilter(pcl::PointCloud<PointType>::Ptr& cloud, float voxel_size)
{
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud);
}

void Relocator::transformFullMap() {

    Eigen::Quaternionf q_inverse = this->q.conjugate();
    Eigen::Vector3f p_inverse = -(q_inverse * this->p);
    pcl::transformPointCloud(*this->full_map_, *this->full_map_transformed_, p_inverse, q_inverse);
    this->voxelGridFilter(this->full_map_transformed_, 0.3f);
}

bool Relocator::relocation() {

    auto start = std::chrono::high_resolution_clock::now();

    bool valid = false;
    if (applyKissMatcher()) {
        valid = applyGICP();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Full Elapsed time ms: " << elapsed_seconds.count()*1000 << std::endl;

    return valid;
}

bool Relocator::applyKissMatcher() {
    const auto& srcVec = convertCloudToVec(*this->source_cloud_);
    const auto& tgtVec = convertCloudToVec(*this->target_map_);
    
    float resolution = 0.3f;
    kiss_matcher::KISSMatcherConfig config(resolution);
    config.use_quatro_ = false;
    config.use_ratio_test_ = true;
    
    kiss_matcher::KISSMatcher matcher(config);
    const auto solution = matcher.estimate(srcVec, tgtVec);
    matcher.print();

    kiss_matcher::KISSMatcherScore score = matcher.getScore();        
    if (score.trans_inliers < this->cfg_.inliers_threshold) {
        std::cout << "KISSMatcher failed to converge" << std::endl;
        std::cout << "Trans inliers: " << score.trans_inliers << "<" << this->cfg_.inliers_threshold << std::endl;
        return false;
    } else {
        std::cout << "KISSMatcher converged" << std::endl;
        std::cout << "Trans inliers: " << score.trans_inliers << ">" << this->cfg_.inliers_threshold << std::endl;
    }
    
    // Compute and store the KISS transformation.
    this->kiss_transformation_ = Eigen::Matrix4f::Identity();
    this->kiss_transformation_.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    this->kiss_transformation_.topRightCorner(3, 1) = solution.translation.cast<float>();
    
    std::cout << this->kiss_transformation_ << std::endl;
    std::cout << "=====================================" << std::endl;
    
    // Transform the source cloud using the KISS transformation.
    pcl::transformPointCloud(*this->source_cloud_, *this->aligned_cloud_, this->kiss_transformation_);
    
    // Update pose from the KISS transformation.
    this->p = this->kiss_transformation_.block<3, 1>(0, 3);
    Eigen::Matrix3f R = this->kiss_transformation_.block<3, 3>(0, 0);
    this->q = Eigen::Quaternionf(R);
    this->q.normalize();
    
    return true;
}


bool Relocator::applyGICP(){

    pcl::PointCloud<pcl::PointXYZI>::Ptr src_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointTypeNano> dummy_;

    this->voxelGridFilter(this->aligned_cloud_, 0.5);
    this->voxelGridFilter(this->target_map_, 0.5);

    pcl::copyPointCloud(*this->aligned_cloud_, *src_);
    pcl::copyPointCloud(*this->target_map_, *dst_);

    double icp_score_threshold = 1000;

    auto start = std::chrono::high_resolution_clock::now();

    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(dummy_);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Elapsed GICP time ms: " << elapsed_seconds.count()*1000 << std::endl;

    double score_ = m_nano_gicp.getFitnessScore();
    std::cout << "Score GICP: " << score_ << std::endl;

    if (m_nano_gicp.hasConverged() && score_ < icp_score_threshold) {
        Eigen::Matrix4f gicpTransformation = m_nano_gicp.getFinalTransformation();
        std::cout << "=============== GICP ==================" << std::endl;
        std::cout << gicpTransformation << std::endl;
        std::cout << "=======================================" << std::endl;


        // The overall transformation from the original source to the target map is:
        Eigen::Matrix4f mergedTransformation = gicpTransformation * this->kiss_transformation_;
        
        // Update pose using the merged transformation.
        this->p = mergedTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3f R = mergedTransformation.block<3, 3>(0, 0);
        this->q = Eigen::Quaternionf(R);
        this->q.normalize();
        
        // Optionally update the aligned cloud with the merged transformation.
        pcl::transformPointCloud(*this->source_cloud_, *this->aligned_cloud_gicp, mergedTransformation);
        return true;
    } 
    else {
        std::cout << "GICP failed to converge or score too high" << std::endl;
        return false;
    }
}
