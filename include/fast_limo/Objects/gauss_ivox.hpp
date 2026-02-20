#pragma once
#include "gauss_octree.h"
#include <unordered_map>

namespace fast_limo {
namespace gauss_mapping {

class GaussianIVox {
public:
    struct HashVec3i {
        size_t operator()(const Eigen::Vector3i& v) const {
            return size_t(((v[0] * 73856093) ^ (v[1] * 19349663) ^ (v[2] * 83492791)));
        }
    };

    GaussianIVox(double v_res = 1.0) : v_res_(v_res), inv_res_(1.0 / v_res) { }

    void addPoint(const PointCov& pt, const CovMat& P) {
        Eigen::Vector3i key = (pt.pos * inv_res_).array().floor().cast<int>();
        if (grids_.find(key) == grids_.end()) {
            Point center = key.cast<double>() * v_res_ + Point::Constant(v_res_ * 0.5);
            grids_.emplace(key, std::make_unique<Octree>(center, v_res_ * 0.5));
        }
        grids_[key]->update(pt, P);
    }

    /**
     * @brief Batch update for the entire scan
     * @param points Vector of points with covariances
     * @param P_curr Current IEKF state covariance
     */
    void addPoints(const VecPointCov& points, 
                   const CovMat& P_curr) {
        
        // Group points by their voxel key
        // Using a temporary map to bucket points locally
        std::unordered_map<Eigen::Vector3i, VecPointCov, HashVec3i> buckets;
        
        for (const auto& pt : points) {
            Eigen::Vector3i key = (pt.pos * inv_res_).array().floor().cast<int>();
            buckets[key].push_back(pt);
        }

        // Update each affected Octree in the global map
        for (auto& bucket : buckets) {
            const Eigen::Vector3i& key = bucket.first;
            auto& points_in_voxel = bucket.second;

            // Create new Octree if voxel is empty
            if (grids_.find(key) == grids_.end()) {
                Point center = key.cast<double>() * v_res_ + Point::Constant(v_res_ * 0.5);
                // The extent of the root octant is half the voxel size
                grids_.emplace(key, std::make_unique<gauss_mapping::Octree>(center, v_res_ * 0.5));
            }

            // Perform batch update on the local octree
            grids_[key]->update(points_in_voxel, P_curr);
        }
    }

    void radiusSearch(const Point& q, double r, GaussianVector& results) {
        double r2 = r * r;
        int r_v = std::ceil(r * inv_res_);
        Eigen::Vector3i ck = (q * inv_res_).array().floor().cast<int>();

        for (int x = -r_v; x <= r_v; ++x) {
            for (int y = -r_v; y <= r_v; ++y) {
                for (int z = -r_v; z <= r_v; ++z) {
                    auto it = grids_.find(ck + Eigen::Vector3i(x, y, z));
                    if (it != grids_.end()) it->second->radiusSearch(it->second->root_.get(), q, r2, results);
                }
            }
        }
    }

    void knnSearch(const Point& q, int k, GaussianVector& results) {
        NeighborHeap heap(k);
        Eigen::Vector3i ck = (q * inv_res_).array().floor().cast<int>();

        // Search 3x3x3 grid around query
        for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {
                for (int z = -1; z <= 1; ++z) {
                    auto it = grids_.find(ck + Eigen::Vector3i(x, y, z));
                    if (it != grids_.end()) {
                        it->second->knnSearch(q, k, heap);
                    }
                }
            }
        }

        // Export results
        results.clear();
        auto final_data = heap.data();
        std::sort(final_data.begin(), final_data.end()); // Sort closest to furthest
        for (const auto& nb : final_data) results.push_back(nb.g);
    }

private:
    double v_res_, inv_res_;
    std::unordered_map<Eigen::Vector3i, std::unique_ptr<Octree>, HashVec3i> grids_;
};

} // namespace gauss_mapping
} // namespace fast_limo