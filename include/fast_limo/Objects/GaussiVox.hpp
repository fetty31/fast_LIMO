#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <list>
#include <memory>
#include <queue>
#include <algorithm>

namespace fast_limo {
namespace gauss_ivox {

using Point = Eigen::Vector3d;
using CovMat = Eigen::Matrix3d;

/**
 * @brief Point representation carrying measurement uncertainty from Kalman Filter
 */
struct PointCov {
    Point pos;
    CovMat cov; 
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 3D Gaussian distribution with Bayesian update logic
 */
struct GaussianModel {
    Point mean;
    CovMat cov;
    int count = 0;

    GaussianModel(const PointCov& pt) {
        mean = pt.pos;
        cov = pt.cov;
        count = 1;
    }

    /**
     * @brief Bayesian Fusion using Kalman Gain via LDLT
     */
    void fuse(const PointCov& pt) {
        // S = Innovation Covariance
        CovMat S = cov + pt.cov;
        
        // Calculate Kalman Gain: K = Sigma_m * S^-1
        // We use LDLT to solve (S * K^T = Sigma_m^T)
        CovMat K = (S.ldlt().solve(cov)).transpose();
        
        // Update State (Mean)
        mean += K * (pt.pos - mean);
        
        // Update Covariance (Joseph Form is more stable but standard form is faster here)
        cov = (CovMat::Identity() - K) * cov;
        
        // Ensure Symmetry (Floating point drift can make 'cov' non-symmetric)
        cov = 0.5 * (cov + cov.transpose().eval());
        
        count++;
    }

    /**
     * @brief Mahalanobis distance using LDLT solver instead of Inverse
     */
    bool checkFit(const PointCov& pt, double threshold) const {
        Point diff = pt.pos - mean;
        
        // Combine map and measurement uncertainty
        CovMat S = cov + pt.cov;
        
        // solve: d2 = diff^T * S^-1 * diff
        double d2 = diff.transpose() * S.ldlt().solve(diff);
        
        return d2 < threshold;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Aligned allocator aliases for Eigen compatibility
using GaussianVector = std::vector<GaussianModel, Eigen::aligned_allocator<GaussianModel>>;

/**
 * @brief Octant node for local voxel refinement
 */
struct Octant {
    Point centroid;
    double extent;
    GaussianVector gaussians;
    std::unique_ptr<Octant[]> children; 

    Octant(const Point& c, double e) : centroid(c), extent(e), children(nullptr) {}
    
    bool isLeaf() const { return children == nullptr; }
    
    void init_children() {
        children = std::unique_ptr<Octant[]>(new Octant[8]{
            {centroid, extent}, {centroid, extent}, {centroid, extent}, {centroid, extent},
            {centroid, extent}, {centroid, extent}, {centroid, extent}, {centroid, extent}
        });

        for (int i = 0; i < 8; ++i) {
            children[i].extent = extent * 0.5;
            children[i].centroid(0) += ((i & 1) ? 0.5 : -0.5) * extent;
            children[i].centroid(1) += ((i & 2) ? 0.5 : -0.5) * extent;
            children[i].centroid(2) += ((i & 4) ? 0.5 : -0.5) * extent;
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GaussianIVox {
public:
    struct Options {
        double voxel_res = 1.0;
        double octree_min_res = 0.1;
        int max_gaussians_per_node = 5;
        double chi_sq_limit = 7.815; 
    };

    explicit GaussianIVox(Options opt) : opt_(opt) {
        inv_res_ = 1.0 / opt_.voxel_res;
    }

    void addPoints(const std::vector<PointCov>& points) {
        for (const auto& pt : points) {
            Eigen::Vector3i key = pos2Key(pt.pos);
            auto it = grids_.find(key);
            if (it == grids_.end()) {
                Point center = key.cast<double>() * opt_.voxel_res + 
                                        Point::Constant(opt_.voxel_res * 0.5);
                auto root = std::make_shared<Octant>(center, opt_.voxel_res * 0.5);
                grids_[key] = root;
                updateRecursive(root.get(), pt);
            } else {
                updateRecursive(it->second.get(), pt);
            }
        }
    }

    void radiusSearch(const Point& query, double radius, GaussianVector& results) {
        double r2 = radius * radius;
        int r_voxels = std::ceil(radius * inv_res_);
        Eigen::Vector3i center_key = pos2Key(query);

        for (int x = -r_voxels; x <= r_voxels; ++x) {
            for (int y = -r_voxels; y <= r_voxels; ++y) {
                for (int z = -r_voxels; z <= r_voxels; ++z) {
                    auto it = grids_.find(center_key + Eigen::Vector3i(x, y, z));
                    if (it != grids_.end()) searchRecursive(it->second.get(), query, r2, results);
                }
            }
        }
    }

    void knn(const Point& query, int k, GaussianVector& neighbors) {
        auto cmp = [](const std::pair<double, GaussianModel>& a, const std::pair<double, GaussianModel>& b) {
            return a.first < b.first;
        };
        std::priority_queue<std::pair<double, GaussianModel>, std::vector<std::pair<double, GaussianModel>>, decltype(cmp)> pq(cmp);

        Eigen::Vector3i center_key = pos2Key(query);
        for (int x = -1; x <= 1; ++x) {
            for (int y = -1; y <= 1; ++y) {
                for (int z = -1; z <= 1; ++z) {
                    auto it = grids_.find(center_key + Eigen::Vector3i(x, y, z));
                    if (it != grids_.end()) knnRecursive(it->second.get(), query, k, pq);
                }
            }
        }
        
        neighbors.clear();
        while(!pq.empty()){
            neighbors.push_back(pq.top().second);
            pq.pop();
        }
        std::reverse(neighbors.begin(), neighbors.end());
    }

    void cleanMap(int min_observations = 5, double max_variance = 0.5) {
        auto it = grids_.begin();
        while (it != grids_.end()) {
            cleanRecursive(it->second.get(), min_observations, max_variance);
            if (it->second->isLeaf() && it->second->gaussians.empty()) it = grids_.erase(it);
            else ++it;
        }
    }

private:
    struct ivec3_hash {
        size_t operator()(const Eigen::Vector3i& v) const {
            return size_t(((v[0] * 73856093) ^ (v[1] * 19349663) ^ (v[2] * 83492791)));
        }
    };

    Eigen::Vector3i pos2Key(const Point& p) const {
        return (p * inv_res_).array().floor().cast<int>();
    }

    void updateRecursive(Octant* node, const PointCov& pt) {
        if (node->isLeaf()) {
            bool fused = false;
            for (auto& g : node->gaussians) {
                if (g.checkFit(pt, opt_.chi_sq_limit)) {
                    g.fuse(pt);
                    fused = true;
                    break;
                }
            }
            if (!fused) node->gaussians.emplace_back(pt);

            if (node->gaussians.size() > (size_t)opt_.max_gaussians_per_node && node->extent > opt_.octree_min_res) {
                split(node);
            }
        } else {
            size_t idx = mortonCode(pt.pos, node->centroid);
            updateRecursive(&node->children[idx], pt);
        }
    }

    void split(Octant* node) {
        node->init_children();
        GaussianVector old_gaussians = std::move(node->gaussians);
        node->gaussians.clear();
        for (const auto& g : old_gaussians) {
            PointCov dummy;
            dummy.pos = g.mean;
            dummy.cov = g.cov;
            updateRecursive(node, dummy);
        }
    }

    void searchRecursive(Octant* node, const Point& q, double r2, GaussianVector& res) {
        if (node->isLeaf()) {
            for (const auto& g : node->gaussians) {
                if ((g.mean - q).squaredNorm() < r2) res.push_back(g);
            }
        } else {
            for (int i = 0; i < 8; ++i) {
                if (overlaps(&node->children[i], q, r2))
                    searchRecursive(&node->children[i], q, r2, res);
            }
        }
    }

    void knnRecursive(Octant* node, const Point& q, int k, auto& pq) {
        if (node->isLeaf()) {
            for (const auto& g : node->gaussians) {
                double d2 = (g.mean - q).squaredNorm();
                if (pq.size() < (size_t)k) pq.push({d2, g});
                else if (d2 < pq.top().first) {
                    pq.pop();
                    pq.push({d2, g});
                }
            }
        } else {
            for (int i = 0; i < 8; ++i) knnRecursive(&node->children[i], q, k, pq);
        }
    }

    void cleanRecursive(Octant* node, int min_obs, double max_var) {
        if (node->isLeaf()) {
            node->gaussians.erase(std::remove_if(node->gaussians.begin(), node->gaussians.end(),
                [&](const GaussianModel& g) {
                    return g.count < min_obs || g.cov.diagonal().maxCoeff() > max_var;
                }), node->gaussians.end());
        } else {
            for (int i = 0; i < 8; ++i) cleanRecursive(&node->children[i], min_obs, max_var);
        }
    }

    bool overlaps(Octant* node, const Point& q, double r2) {
        double dist_sq = 0.0;
        for (int i = 0; i < 3; ++i) {
            double min_b = node->centroid[i] - node->extent;
            double max_b = node->centroid[i] + node->extent;
            if (q[i] < min_b) dist_sq += std::pow(min_b - q[i], 2);
            else if (q[i] > max_b) dist_sq += std::pow(q[i] - max_b, 2);
        }
        return dist_sq <= r2;
    }

    inline size_t mortonCode(const Point& p, const Point& centroid) {
        size_t out(0);
        if (p(0) > centroid(0)) out |= 1;
        if (p(1) > centroid(1)) out |= 2;
        if (p(2) > centroid(2)) out |= 4;
        return out;
    }

    std::unordered_map<Eigen::Vector3i, std::shared_ptr<Octant>, ivec3_hash> grids_;
    Options opt_;
    double inv_res_;
};

} // namespace gauss_ivox
} // namespace fast_limo