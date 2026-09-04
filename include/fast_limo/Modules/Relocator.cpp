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

#include <limits>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

namespace {

struct PriorGICPResult {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    bool converged = false;
    double fitness_score = std::numeric_limits<double>::infinity();
};

bool validRigidTransform(const Eigen::Matrix4f& transformation) {
    if (!transformation.allFinite()) return false;

    const Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    const float determinant = rotation.determinant();
    const float orthogonality_error =
        (rotation.transpose() * rotation - Eigen::Matrix3f::Identity()).norm();

    return std::abs(determinant - 1.0f) < 0.05f && orthogonality_error < 0.1f;
}

pcl::PointCloud<PointType>::Ptr downsampleCloud(
    const pcl::PointCloud<PointType>::Ptr& input,
    float voxel_size)
{
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>);
    if (!input || input->empty()) return output;

    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.filter(*output);
    return output;
}

pcl::PointCloud<PointType>::Ptr cropCloudAroundPrior(
    const pcl::PointCloud<PointType>::Ptr& cloud_in_map,
    const Eigen::Vector3f& prior_position,
    float crop_size)
{
    pcl::PointCloud<PointType>::Ptr cropped(new pcl::PointCloud<PointType>);
    if (!cloud_in_map || cloud_in_map->empty() || crop_size <= 0.0f) return cropped;

    pcl::CropBox<PointType> crop;
    crop.setInputCloud(cloud_in_map);
    crop.setMin(Eigen::Vector4f(
        prior_position.x() - crop_size,
        prior_position.y() - crop_size,
        prior_position.z() - crop_size,
        1.0f));
    crop.setMax(Eigen::Vector4f(
        prior_position.x() + crop_size,
        prior_position.y() + crop_size,
        prior_position.z() + crop_size,
        1.0f));
    crop.filter(*cropped);
    return cropped;
}

PriorGICPResult runGICP(
    const pcl::PointCloud<PointType>::Ptr& source,
    const pcl::PointCloud<PointType>::Ptr& target,
    const Eigen::Matrix4f& guess,
    float max_correspondence,
    int maximum_iterations)
{
    PriorGICPResult result;
    if (!source || !target || source->size() < 20 || target->size() < 20) {
        return result;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_xyz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*source, *source_xyz);
    pcl::copyPointCloud(*target, *target_xyz);

    nano_gicp::NanoGICP<PointTypeNano, PointTypeNano> registration;
    registration.setMaxCorrespondenceDistance(max_correspondence);
    registration.setNumThreads(10);
    registration.setCorrespondenceRandomness(20);
    registration.setMaximumIterations(maximum_iterations);
    registration.setTransformationEpsilon(0.01);
    registration.setEuclideanFitnessEpsilon(0.01);
    registration.setRANSACIterations(5);
    registration.setRANSACOutlierRejectionThreshold(1.0);

    pcl::PointCloud<PointTypeNano> output;
    registration.setInputSource(source_xyz);
    registration.calculateSourceCovariances();
    registration.setInputTarget(target_xyz);
    registration.calculateTargetCovariances();
    registration.align(output, guess);

    result.converged = registration.hasConverged();
    if (!result.converged) return result;

    result.transformation = registration.getFinalTransformation();
    if (!validRigidTransform(result.transformation)) {
        result.converged = false;
        return result;
    }

    result.fitness_score = registration.getFitnessScore();
    return result;
}

}  // namespace

// class fast_limo::Relocator
// public

std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<PointType>& cloud) {
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
    prior_debug_source_map_.reset(new pcl::PointCloud<PointType>);
    prior_debug_target_map_.reset(new pcl::PointCloud<PointType>);

    p = Eigen::Vector3f::Zero();
    q = Eigen::Quaternionf::Identity();


    // Initialize NanoGICP with default parameters
    m_nano_gicp = nano_gicp::NanoGICP<PointTypeNano, PointTypeNano>();
    m_nano_gicp.setMaxCorrespondenceDistance(2.0);
    m_nano_gicp.setNumThreads(10);
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

Eigen::Vector3f Relocator::get_pose() {
    std::lock_guard<std::mutex> lock(mutex_);
    return p;
}

Eigen::Quaternionf Relocator::get_orientation() {
    std::lock_guard<std::mutex> lock(mutex_);
    return q;
}

bool Relocator::takePriorDebugClouds(
    pcl::PointCloud<PointType>::Ptr& source_in_map,
    pcl::PointCloud<PointType>::Ptr& target_in_map)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!prior_debug_clouds_ready_) return false;

    source_in_map.reset(new pcl::PointCloud<PointType>(*prior_debug_source_map_));
    target_in_map.reset(new pcl::PointCloud<PointType>(*prior_debug_target_map_));
    prior_debug_clouds_ready_ = false;
    return true;
}

bool Relocator::enough_distance_traveled() {
    const float threshold = cfg_.mode
        ? cfg_.prior_distance_threshold
        : cfg_.distance_threshold;
    std::cout << "Distance traveled: " << this->distance_traveled << " / " << threshold << std::endl;
    return this->distance_traveled > threshold;
}

void Relocator::updateInitialPose(const Eigen::Matrix4f& map_to_base,
                                  const Eigen::Matrix4f& odom_to_base){
    std::lock_guard<std::mutex> lock(mutex_);

    // Start the accumulation window when the first valid prior arrives.
    // Later prior messages refresh the guess without discarding the submap.
    if (!this->recived_estimated_pose) {
        this->distance_traveled = 0.0f;
        this->source_cloud_->clear();
        this->last_x = odom_to_base(0, 3);
        this->last_y = odom_to_base(1, 3);
    }

    this->initial_map_to_base_ = map_to_base;
    this->initial_odom_to_base_ = odom_to_base;
    this->init_state_[0] = map_to_base(0, 3);
    this->init_state_[1] = map_to_base(1, 3);
    this->init_state_[2] = map_to_base(2, 3);
    this->recived_estimated_pose = true; 
}

void Relocator::reset() {
    this->distance_traveled = 0.0f;
    this->source_cloud_->clear();
    this->target_map_->clear();
    this->aligned_cloud_->clear();
    this->aligned_cloud_gicp->clear();
    this->recived_estimated_pose = false;
    this->last_x = std::nan("");
    p = Eigen::Vector3f::Zero();
    q = Eigen::Quaternionf::Identity();
}

void Relocator::updateCloud(pcl::PointCloud<PointType>::Ptr& pc) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Prior mode only accumulates scans after a valid /initialpose.
    if(cfg_.mode && !recived_estimated_pose) return;

    // Accumulate the point cloud
    *this->source_cloud_ += *pc;

    // Check if has been traveled enough distance
    if(!enough_distance_traveled()) return;
    
    std::cout << "Starting Relocating..." << std::endl;

    // Global mode uses the full target map. Prior mode crops a fixed box
    // centered at the synchronized /initialpose position.
    if (!cfg_.mode) {
        pcl::copyPointCloud(*this->full_map_, *this->target_map_);
    }

    this->relocated = this->relocation();
    if(this->relocated) this->transformFullMap();
    else this->reset();
}

void Relocator::updateState(fast_limo::State& st) {
    std::lock_guard<std::mutex> lock(mutex_);

    float current_x = st.p(0);
    float current_y = st.p(1);
    
    if(std::isnan(last_x)){
        last_x = current_x;
        last_y = current_y;
        return;
    } 
        
    float dx = current_x - last_x;
    float dy = current_y - last_y;
    float delta_distance = std::sqrt(dx * dx + dy * dy);
    
    if(delta_distance > 0.1f){
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
    if (cfg_.mode) {
        valid = applyPriorGICP();
    } else if (applyKissMatcher()) {
        valid = applyGICP();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Full Elapsed time ms: " << elapsed_seconds.count()*1000 << std::endl;

    return valid;
}

bool Relocator::applyPriorGICP() {
    if (!recived_estimated_pose || source_cloud_->empty() || full_map_->empty()) {
        std::cout << "Prior GICP cannot start: missing prior, source cloud or map" << std::endl;
        return false;
    }

    const Eigen::Matrix4f map_to_odom_guess =
        initial_map_to_base_ * initial_odom_to_base_.inverse();
    if (!validRigidTransform(map_to_odom_guess)) {
        std::cout << "Prior GICP cannot start: invalid initial transform" << std::endl;
        return false;
    }

    pcl::PointCloud<PointType>::Ptr source =
        downsampleCloud(source_cloud_, cfg_.prior_voxel);

    const Eigen::Vector3f prior_position =
        initial_map_to_base_.block<3, 1>(0, 3);

    std::cout
        << "Prior crop box: center=["
        << prior_position.x() << ", "
        << prior_position.y() << ", "
        << prior_position.z() << "], crop_size="
        << cfg_.prior_crop_size
        << std::endl;

    // Transform the source provisionally to map, apply exactly the same crop
    // box as the target, and transform it back to odom for GICP. This preserves
    // the original registration convention: source=odom, target=map, guess=map<-odom.
    pcl::PointCloud<PointType>::Ptr source_in_map(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(
        *source,
        *source_in_map,
        map_to_odom_guess);

    const std::size_t source_size_before_crop = source_in_map->size();
    pcl::PointCloud<PointType>::Ptr cropped_source_in_map =
        cropCloudAroundPrior(
            source_in_map,
            prior_position,
            cfg_.prior_crop_size);

    pcl::copyPointCloud(*cropped_source_in_map, *prior_debug_source_map_);
    prior_debug_target_map_->clear();
    prior_debug_clouds_ready_ = true;

    const Eigen::Matrix4f odom_to_map_guess = map_to_odom_guess.inverse();
    pcl::transformPointCloud(
        *cropped_source_in_map,
        *source,
        odom_to_map_guess);

    std::cout
        << "Prior source crop: before=" << source_size_before_crop
        << ", after=" << source->size()
        << std::endl;

    if (source->size() < 20) {
        std::cout << "Prior GICP source cloud is too small after cropping" << std::endl;
        return false;
    }

    pcl::PointCloud<PointType>::Ptr cropped_map =
        cropCloudAroundPrior(
            full_map_,
            prior_position,
            cfg_.prior_crop_size);

    pcl::PointCloud<PointType>::Ptr target =
        downsampleCloud(cropped_map, cfg_.prior_voxel);
    pcl::copyPointCloud(*target, *prior_debug_target_map_);

    if (cropped_map->size() < 20) {
        std::cout << "Prior GICP target map is too small after cropping" << std::endl;
        return false;
    }
    if (target->size() < 20) {
        std::cout << "Prior GICP target map is too small after downsampling" << std::endl;
        return false;
    }

    const PriorGICPResult result = runGICP(
        source,
        target,
        map_to_odom_guess,
        cfg_.prior_max_correspondence,
        cfg_.prior_max_iterations);

    std::cout
        << "Prior GICP: converged=" << result.converged
        << ", fitness=" << result.fitness_score
        << ", maximum_fitness=" << cfg_.prior_max_fitness_score
        << std::endl;

    if (!result.converged ||
        !std::isfinite(result.fitness_score) ||
        result.fitness_score > cfg_.prior_max_fitness_score)
    {
        std::cout << "Prior GICP rejected: not converged or fitness too high" << std::endl;
        return false;
    }

    this->p = result.transformation.block<3, 1>(0, 3);
    this->q = Eigen::Quaternionf(result.transformation.block<3, 3>(0, 0));
    this->q.normalize();
    pcl::transformPointCloud(
        *this->source_cloud_,
        *this->aligned_cloud_gicp,
        result.transformation);

    std::cout
        << "Prior GICP accepted with fitness=" << result.fitness_score
        << std::endl;
    return true;
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
        return false;
    } else std::cout << "KISSMatcher converged" << std::endl;
    
    // Compute and store the KISS transformation.
    this->kiss_transformation_ = Eigen::Matrix4f::Identity();
    this->kiss_transformation_.block<3, 3>(0, 0) = solution.rotation.cast<float>();
    this->kiss_transformation_.topRightCorner(3, 1) = solution.translation.cast<float>();
    
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

    std::cout << "Aligned cloud size: " << this->aligned_cloud_->size() << std::endl;

    // Filter the map around the pose found by KISSMatcher
    init_state_[0] = p[0];
    init_state_[1] = p[1];
    this->passThroughFilter(this->target_map_, 20);
    
    pcl::copyPointCloud(*this->aligned_cloud_, *src_);
    pcl::copyPointCloud(*this->target_map_, *dst_);
        
    auto start = std::chrono::high_resolution_clock::now();
    
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(dummy_);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "=============== GICP ==================" << std::endl;
    std::cout << "Elapsed GICP time ms: " << elapsed_seconds.count()*1000 << std::endl;

    double score_ = m_nano_gicp.getFitnessScore();
    std::cout << "Score GICP: " << score_ << std::endl;

    if (m_nano_gicp.hasConverged() && score_ < cfg_.score) {
        Eigen::Matrix4f gicpTransformation = m_nano_gicp.getFinalTransformation();
        
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
    std::cout << "=======================================" << std::endl;
}
