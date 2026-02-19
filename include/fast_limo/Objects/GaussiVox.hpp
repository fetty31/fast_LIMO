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

/**
 * @brief Point representation carrying measurement uncertainty from Kalman Filter
 */
struct PointCov {
    Eigen::Vector3d pos;
    Eigen::Matrix3d cov; 
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 3D Gaussian distribution with Bayesian update logic
 */
struct GaussianModel {
    Eigen::Vector3d mean;
    Eigen::Matrix3d cov;
    Eigen::Matrix3d info; 
    int count = 0;

    GaussianModel(const PointCov& pt) {
        mean = pt.pos;
        cov = pt.cov;
        count = 1;
        updateInverse();
    }

    void updateInverse() {
        // Pseudo-inverse for numerical stability
        info = (cov + Eigen::Matrix3d::Identity() * 1e-6).inverse();
    }

    void fuse(const PointCov& pt) {
        // Kalman-style Bayesian fusion
        Eigen::Matrix3d K = cov * (cov + pt.cov).inverse();
        mean = mean + K * (pt.pos - mean);
        cov = (Eigen::Matrix3d::Identity() - Eigen::Matrix3d(K)) * cov;
        count++;
        updateInverse();
    }

    bool checkFit(const PointCov& pt, double threshold) const {
        Eigen::Vector3d diff = pt.pos - mean;
        double dist_sq = diff.transpose() * info * diff;
        return dist_sq < threshold;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Aligned allocator aliases for Eigen compatibility
using GaussianVector = std::vector<GaussianModel, Eigen::aligned_allocator<GaussianModel>>;

/**
 * @brief Octant node for local voxel refinement
 */
struct Octant {
    Eigen::Vector3d centroid;
    double extent;
    GaussianVector gaussians;
    std::unique_ptr<Octant[]> children; 

    Octant(const Eigen::Vector3d& c, double e) : centroid(c), extent(e), children(nullptr) {}
    
    bool isLeaf() const { return children == nullptr; }
    
    void init_children() {
        children = std::unique_ptr<Octant[]>(new Octant[8]{
            {centroid, extent}, {centroid, extent}, {centroid, extent}, {centroid, extent},
            {centroid, extent}, {centroid, extent}, {centroid, extent}, {centroid, extent}
        });

        for (int i = 0; i < 8; ++i) {
            children[i].extent = extent * 0.5;
            children[i].centroid.x() += ((i & 1) ? 0.5 : -0.5) * extent;
            children[i].centroid.y() += ((i & 2) ? 0.5 : -0.5) * extent;
            children[i].centroid.z() += ((i & 4) ? 0.5 : -0.5) * extent;
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

    void AddPoints(const std::vector<PointCov>& points) {
        for (const auto& pt : points) {
            Eigen::Vector3i key = pos2Key(pt.pos);
            auto it = grids_.find(key);
            if (it == grids_.end()) {
                Eigen::Vector3d center = key.cast<double>() * opt_.voxel_res + 
                                        Eigen::Vector3d::Constant(opt_.voxel_res * 0.5);
                auto root = std::make_shared<Octant>(center, opt_.voxel_res * 0.5);
                grids_[key] = root;
                updateRecursive(root.get(), pt);
            } else {
                updateRecursive(it->second.get(), pt);
            }
        }
    }

    void RadiusSearch(const Eigen::Vector3d& query, double radius, GaussianVector& results) {
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

    void KNN(const Eigen::Vector3d& query, int k, GaussianVector& neighbors) {
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

    void CleanMap(int min_observations = 5, double max_variance = 0.5) {
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

    Eigen::Vector3i pos2Key(const Eigen::Vector3d& p) const {
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
            int idx = 0;
            if (pt.pos.x() > node->centroid.x()) idx |= 1;
            if (pt.pos.y() > node->centroid.y()) idx |= 2;
            if (pt.pos.z() > node->centroid.z()) idx |= 4;
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

    void searchRecursive(Octant* node, const Eigen::Vector3d& q, double r2, GaussianVector& res) {
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

    void knnRecursive(Octant* node, const Eigen::Vector3d& q, int k, auto& pq) {
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

    bool overlaps(Octant* node, const Eigen::Vector3d& q, double r2) {
        double dist_sq = 0.0;
        for (int i = 0; i < 3; ++i) {
            double min_b = node->centroid[i] - node->extent;
            double max_b = node->centroid[i] + node->extent;
            if (q[i] < min_b) dist_sq += std::pow(min_b - q[i], 2);
            else if (q[i] > max_b) dist_sq += std::pow(q[i] - max_b, 2);
        }
        return dist_sq <= r2;
    }

    std::unordered_map<Eigen::Vector3i, std::shared_ptr<Octant>, ivec3_hash> grids_;
    Options opt_;
    double inv_res_;
};

} // namespace gauss_ivox
} // namespace fast_limo