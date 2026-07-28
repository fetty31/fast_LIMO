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

#include <algorithm>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace {

constexpr float kPi = 3.14159265358979323846f;

struct LocalCandidate {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    bool converged = false;
    int inliers = 0;
    float inlier_ratio = 0.0f;
    float rmse = std::numeric_limits<float>::infinity();
    float quality = std::numeric_limits<float>::infinity();
    float position_error = std::numeric_limits<float>::infinity();
    float z_error = std::numeric_limits<float>::infinity();
};

float normalizeAngle(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

float yawFromRotation(const Eigen::Matrix3f& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

void rollPitchFromRotation(const Eigen::Matrix3f& rotation, float& roll, float& pitch) {
    pitch = std::asin(std::clamp(-rotation(2, 0), -1.0f, 1.0f));
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
}

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

pcl::PointCloud<PointType>::Ptr cropMapFromSourceBoundingBox(
    const pcl::PointCloud<PointType>::Ptr& full_map,
    const pcl::PointCloud<PointType>::Ptr& source,
    const Eigen::Matrix4f& guess,
    float margin)
{
    pcl::PointCloud<PointType>::Ptr cropped(new pcl::PointCloud<PointType>);
    if (!full_map || !source || full_map->empty() || source->empty()) return cropped;

    pcl::PointCloud<PointType>::Ptr transformed_source(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*source, *transformed_source, guess);

    PointType min_point;
    PointType max_point;
    pcl::getMinMax3D(*transformed_source, min_point, max_point);

    pcl::CropBox<PointType> crop;
    crop.setInputCloud(full_map);
    crop.setMin(Eigen::Vector4f(
        min_point.x - margin,
        min_point.y - margin,
        min_point.z - margin,
        1.0f));
    crop.setMax(Eigen::Vector4f(
        max_point.x + margin,
        max_point.y + margin,
        max_point.z + margin,
        1.0f));
    crop.filter(*cropped);
    return cropped;
}

void evaluateAlignment(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const Eigen::Matrix4f& transformation,
    float max_correspondence,
    LocalCandidate& candidate)
{
    candidate.inliers = 0;
    candidate.inlier_ratio = 0.0f;
    candidate.rmse = std::numeric_limits<float>::infinity();
    candidate.quality = std::numeric_limits<float>::infinity();

    if (!source || !target || source->empty() || target->empty()) return;

    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(target);

    const float max_distance_squared = max_correspondence * max_correspondence;
    double squared_error_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);

    for (const auto& point : source->points) {
        Eigen::Vector4f homogeneous(point.x, point.y, point.z, 1.0f);
        homogeneous = transformation * homogeneous;

        pcl::PointXYZI transformed;
        transformed.x = homogeneous.x();
        transformed.y = homogeneous.y();
        transformed.z = homogeneous.z();

        if (tree.nearestKSearch(transformed, 1, indices, squared_distances) == 1 &&
            squared_distances[0] <= max_distance_squared)
        {
            ++candidate.inliers;
            squared_error_sum += squared_distances[0];
        }
    }

    candidate.inlier_ratio =
        static_cast<float>(candidate.inliers) / static_cast<float>(source->size());

    if (candidate.inliers > 0) {
        candidate.rmse =
            static_cast<float>(std::sqrt(squared_error_sum / candidate.inliers));
        candidate.quality =
            candidate.rmse / std::max(candidate.inlier_ratio, 0.05f);
    }
}

LocalCandidate runGICP(
    const pcl::PointCloud<PointType>::Ptr& source,
    const pcl::PointCloud<PointType>::Ptr& target,
    const Eigen::Matrix4f& guess,
    float max_correspondence,
    int maximum_iterations)
{
    LocalCandidate candidate;
    if (!source || !target || source->size() < 20 || target->size() < 20) {
        return candidate;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_xyz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*source, *source_xyz);
    pcl::copyPointCloud(*target, *target_xyz);

    nano_gicp::NanoGICP<PointTypeNano, PointTypeNano> registration;
    registration.setMaxCorrespondenceDistance(max_correspondence);
    registration.setNumThreads(10);
    registration.setCorrespondenceRandomness(15);
    registration.setMaximumIterations(maximum_iterations);
    registration.setTransformationEpsilon(1.0e-4);
    registration.setEuclideanFitnessEpsilon(1.0e-5);
    registration.setRANSACIterations(5);
    registration.setRANSACOutlierRejectionThreshold(max_correspondence);

    pcl::PointCloud<PointTypeNano> output;
    registration.setInputSource(source_xyz);
    registration.calculateSourceCovariances();
    registration.setInputTarget(target_xyz);
    registration.calculateTargetCovariances();
    registration.align(output, guess);

    candidate.converged = registration.hasConverged();
    if (!candidate.converged) return candidate;

    candidate.transformation = registration.getFinalTransformation();
    if (!validRigidTransform(candidate.transformation)) {
        candidate.converged = false;
        return candidate;
    }

    evaluateAlignment(
        source_xyz,
        target_xyz,
        candidate.transformation,
        max_correspondence,
        candidate);

    return candidate;
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

bool Relocator::enough_distance_traveled() {
    std::cout << "Distance traveled: " << this->distance_traveled << " / " << this->cfg_.distance_threshold << std::endl;
    return this->distance_traveled > this->cfg_.distance_threshold;
}

void Relocator::updateInitialPose(const Eigen::Vector3f& map_position,
                                  const Eigen::Matrix4f& odom_to_base){
    std::lock_guard<std::mutex> lock(mutex_);
    this->initial_map_position_ = map_position;
    this->initial_odom_to_base_ = odom_to_base;
    this->init_state_[0] = map_position.x();
    this->init_state_[1] = map_position.y();
    this->init_state_[2] = map_position.z();
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

    // Accumulate the point cloud
    *this->source_cloud_ += *pc;

    // Check if has been traveled enough distance
    if(!enough_distance_traveled()) return;
    
    // Check if we have received the initial pose (if mode is true)
    if(cfg_.mode && !recived_estimated_pose) return;

    std::cout << "Starting Relocating..." << std::endl;

    // Global mode uses the full target map. Local mode crops full_map_ from
    // the transformed source bounding box independently for every yaw guess.
    if (!cfg_.mode) {
        pcl::copyPointCloud(*this->full_map_, *this->target_map_);
    }

    this->relocated = this->relocation(); // Apply KISSMatcher and GICP 
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
        valid = applyLocalGICP();
    } else if (applyKissMatcher()) {
        valid = applyGICP();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Full Elapsed time ms: " << elapsed_seconds.count()*1000 << std::endl;

    return valid;
}

bool Relocator::applyLocalGICP() {
    if (!recived_estimated_pose || source_cloud_->empty() || full_map_->empty()) {
        std::cout << "Local GICP cannot start: missing prior, source cloud or map" << std::endl;
        return false;
    }

    const float yaw_step_rad = cfg_.local_yaw_step_deg * kPi / 180.0f;
    if (!(yaw_step_rad > 0.0f) || yaw_step_rad > 2.0f * kPi) {
        std::cout << "Invalid local_yaw_step_deg" << std::endl;
        return false;
    }

    pcl::PointCloud<PointType>::Ptr coarse_source =
        downsampleCloud(source_cloud_, cfg_.local_coarse_voxel);
    if (coarse_source->size() < 20) {
        std::cout << "Local GICP source cloud is too small after coarse downsampling" << std::endl;
        return false;
    }

    const Eigen::Matrix3f odom_base_rotation =
        initial_odom_to_base_.block<3, 3>(0, 0);
    const float odom_base_yaw = yawFromRotation(odom_base_rotation);
    const Eigen::Matrix3f odom_base_roll_pitch =
        Eigen::AngleAxisf(-odom_base_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
        odom_base_rotation;

    std::vector<LocalCandidate> coarse_candidates;
    const int number_of_hypotheses =
        std::max(1, static_cast<int>(std::ceil(2.0f * kPi / yaw_step_rad)));
    coarse_candidates.reserve(number_of_hypotheses);

    const int coarse_min_inliers = std::max(50, cfg_.local_min_inliers / 4);
    const float coarse_min_ratio = std::min(0.15f, cfg_.local_min_inlier_ratio * 0.5f);
    const float coarse_max_rmse = std::max(
        cfg_.local_coarse_max_correspondence * 0.75f,
        cfg_.local_max_rmse * 2.0f);

    for (int i = 0; i < number_of_hypotheses; ++i) {
        const float map_base_yaw = i * yaw_step_rad;

        Eigen::Matrix4f map_to_base_guess = Eigen::Matrix4f::Identity();
        map_to_base_guess.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(map_base_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
            odom_base_roll_pitch;
        map_to_base_guess.block<3, 1>(0, 3) = initial_map_position_;

        const Eigen::Matrix4f map_to_odom_guess =
            map_to_base_guess * initial_odom_to_base_.inverse();

        pcl::PointCloud<PointType>::Ptr cropped_map =
            cropMapFromSourceBoundingBox(
                full_map_,
                coarse_source,
                map_to_odom_guess,
                cfg_.local_crop_margin);
        pcl::PointCloud<PointType>::Ptr coarse_target =
            downsampleCloud(cropped_map, cfg_.local_coarse_voxel);

        LocalCandidate candidate = runGICP(
            coarse_source,
            coarse_target,
            map_to_odom_guess,
            cfg_.local_coarse_max_correspondence,
            40);

        if (!candidate.converged ||
            candidate.inliers < coarse_min_inliers ||
            candidate.inlier_ratio < coarse_min_ratio ||
            candidate.rmse > coarse_max_rmse)
        {
            continue;
        }

        const Eigen::Vector3f candidate_map_position =
            (candidate.transformation * initial_odom_to_base_).block<3, 1>(0, 3);
        candidate.position_error =
            (candidate_map_position.head<2>() - initial_map_position_.head<2>()).norm();
        candidate.z_error =
            std::abs(candidate_map_position.z() - initial_map_position_.z());

        coarse_candidates.push_back(candidate);
    }

    if (coarse_candidates.empty()) {
        std::cout << "Local GICP rejected every coarse yaw hypothesis" << std::endl;
        return false;
    }

    std::sort(
        coarse_candidates.begin(),
        coarse_candidates.end(),
        [](const LocalCandidate& a, const LocalCandidate& b) {
            return a.quality < b.quality;
        });

    pcl::PointCloud<PointType>::Ptr fine_source =
        downsampleCloud(source_cloud_, cfg_.local_fine_voxel);
    std::vector<LocalCandidate> fine_candidates;

    const int refine_count = std::min(
        cfg_.local_refine_candidates,
        static_cast<int>(coarse_candidates.size()));

    for (int i = 0; i < refine_count; ++i) {
        const Eigen::Matrix4f& coarse_guess = coarse_candidates[i].transformation;

        pcl::PointCloud<PointType>::Ptr cropped_map =
            cropMapFromSourceBoundingBox(
                full_map_,
                fine_source,
                coarse_guess,
                cfg_.local_crop_margin);
        pcl::PointCloud<PointType>::Ptr fine_target =
            downsampleCloud(cropped_map, cfg_.local_fine_voxel);

        LocalCandidate candidate = runGICP(
            fine_source,
            fine_target,
            coarse_guess,
            cfg_.local_fine_max_correspondence,
            64);

        if (!candidate.converged || !validRigidTransform(candidate.transformation)) {
            continue;
        }

        const Eigen::Matrix4f map_to_base =
            candidate.transformation * initial_odom_to_base_;
        const Eigen::Vector3f candidate_map_position =
            map_to_base.block<3, 1>(0, 3);
        candidate.position_error =
            (candidate_map_position.head<2>() - initial_map_position_.head<2>()).norm();
        candidate.z_error =
            std::abs(candidate_map_position.z() - initial_map_position_.z());

        float roll = 0.0f;
        float pitch = 0.0f;
        rollPitchFromRotation(
            candidate.transformation.block<3, 3>(0, 0),
            roll,
            pitch);
        const float max_roll_pitch =
            std::max(std::abs(roll), std::abs(pitch)) * 180.0f / kPi;

        const bool accepted =
            candidate.inliers >= cfg_.local_min_inliers &&
            candidate.inlier_ratio >= cfg_.local_min_inlier_ratio &&
            candidate.rmse <= cfg_.local_max_rmse &&
            candidate.position_error <= cfg_.local_max_position_correction &&
            candidate.z_error <= cfg_.local_max_z_correction &&
            max_roll_pitch <= cfg_.local_max_roll_pitch_deg;

        std::cout
            << "Local GICP candidate: inliers=" << candidate.inliers
            << ", ratio=" << candidate.inlier_ratio
            << ", rmse=" << candidate.rmse
            << ", gps_xy_error=" << candidate.position_error
            << ", gps_z_error=" << candidate.z_error
            << ", roll_pitch=" << max_roll_pitch
            << ", accepted=" << accepted
            << std::endl;

        if (accepted) fine_candidates.push_back(candidate);
    }

    if (fine_candidates.empty()) {
        std::cout << "Local GICP rejected every refined candidate" << std::endl;
        return false;
    }

    std::sort(
        fine_candidates.begin(),
        fine_candidates.end(),
        [](const LocalCandidate& a, const LocalCandidate& b) {
            return a.quality < b.quality;
        });

    // Merge candidates that converged to the same physical solution. Adjacent
    // yaw hypotheses often fall into the same GICP basin and must not be
    // mistaken for ambiguous independent solutions.
    std::vector<LocalCandidate> unique_candidates;
    for (const auto& candidate : fine_candidates) {
        const Eigen::Matrix4f candidate_map_to_base =
            candidate.transformation * initial_odom_to_base_;
        const Eigen::Vector3f candidate_position =
            candidate_map_to_base.block<3, 1>(0, 3);
        const float candidate_yaw =
            yawFromRotation(candidate.transformation.block<3, 3>(0, 0));

        bool same_solution = false;
        for (const auto& unique : unique_candidates) {
            const Eigen::Matrix4f unique_map_to_base =
                unique.transformation * initial_odom_to_base_;
            const Eigen::Vector3f unique_position =
                unique_map_to_base.block<3, 1>(0, 3);
            const float unique_yaw =
                yawFromRotation(unique.transformation.block<3, 3>(0, 0));

            const float position_difference =
                (candidate_position.head<2>() - unique_position.head<2>()).norm();
            const float yaw_difference =
                std::abs(normalizeAngle(candidate_yaw - unique_yaw)) * 180.0f / kPi;

            if (position_difference < 0.75f && yaw_difference < 5.0f) {
                same_solution = true;
                break;
            }
        }

        if (!same_solution) unique_candidates.push_back(candidate);
    }

    const LocalCandidate& best = unique_candidates.front();
    if (unique_candidates.size() > 1) {
        const LocalCandidate& second = unique_candidates[1];
        const float required_second_quality =
            best.quality * (1.0f + cfg_.local_min_solution_separation);
        if (second.quality <= required_second_quality) {
            std::cout
                << "Local GICP rejected an ambiguous result: best_quality="
                << best.quality
                << ", second_quality=" << second.quality
                << std::endl;
            return false;
        }
    }

    this->p = best.transformation.block<3, 1>(0, 3);
    this->q = Eigen::Quaternionf(best.transformation.block<3, 3>(0, 0));
    this->q.normalize();
    pcl::transformPointCloud(
        *this->source_cloud_,
        *this->aligned_cloud_gicp,
        best.transformation);

    std::cout
        << "Local GICP accepted: inliers=" << best.inliers
        << ", ratio=" << best.inlier_ratio
        << ", rmse=" << best.rmse
        << ", quality=" << best.quality
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
