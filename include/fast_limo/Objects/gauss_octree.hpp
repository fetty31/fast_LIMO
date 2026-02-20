#pragma once

#include "gauss_common.h"
#include <queue>

namespace fast_limo {
namespace gauss_mapping {

// Morton-order based neighbor lookup table for optimized traversal
static const int ordered_indices[8][7] = {
    {1, 2, 4, 3, 5, 6, 7}, {0, 3, 5, 2, 4, 7, 6}, {3, 0, 6, 1, 7, 4, 5}, {2, 1, 7, 0, 6, 5, 4},
    {5, 6, 0, 7, 1, 2, 3}, {4, 7, 1, 6, 0, 3, 2}, {7, 4, 2, 5, 3, 0, 1}, {6, 5, 3, 4, 2, 1, 0}
};

class Octree {
public:
    Octree(const Point& center, double extent, size_t max_g = 5, double min_e = 0.1)
        : max_gaussians_(max_g), min_extent_(min_e) {
        root_ = std::make_unique<Octant>(center, extent);
    }

    void update(const PointCov& pt, const CovMat& P_curr) {
        updateRecursive(root_.get(), pt, P_curr);
    }

    void update(const std::vector<PointCov>& points, const CovMat& P_curr) {
        updateMultiRecursive(root_.get(), points, P_curr);
    }

    void radiusSearch(Octant* node, const Point& q, double r2, GaussianVector& results) const {
        if (node->isLeaf()) {
            for (const auto& g : node->gaussians) {
                if ((g.mean - q).squaredNorm() < r2) results.push_back(g);
            }
        } else {
            for (int i = 0; i < 8; ++i) {
                if (boxOverlap(node->children[i], q, r2))
                    radiusSearch(&node->children[i], q, r2, results);
            }
        }
    }

    void knnSearch(const Point& q, int k, NeighborHeap& heap) const {
        if (!root_) return;
        knnRecursive(root_.get(), q, heap);
    }

private:
    std::unique_ptr<Octant> root_;
    size_t max_gaussians_;
    double min_extent_;

    void updateMultiRecursive(Octant* octant, const VecPointCov& points, const CovMat& P_curr) {
        if (octant->isLeaf()) {
            // Add all points to existing Gaussians or create new ones
            for (const auto& pt : points) {
                bool fused = false;
                for (auto& g : octant->gaussians) {
                    // Check fit using the combined uncertainty logic
                    if (g.checkFit(pt.pos, pt.cov)) {
                        // Use the smallest eigenvector as the normal for AKF-LIO fusion
                        Eigen::SelfAdjointEigenSolver<CovMat> es(g.cov);
                        g.fuseAdaptive(pt, P_curr, es.eigenvectors().col(0));
                        fused = true;
                        break;
                    }
                }
                if (!fused) {
                    octant->gaussians.emplace_back(pt);
                }
            }

            // Merge overlapping Gaussians after the whole batch is in
            mergeGaussiansInOctant(octant);

            // Split if necessary
            if (octant->gaussians.size() > max_gaussians_ && octant->extent > min_extent_) {
                splitOctant(octant, P_curr);
            }
        } else {
            // Internal Node logic: Batch distribute points to children
            std::vector<std::vector<PointCov>> child_buckets(8);
            for (const auto& pt : points) {
                child_buckets[getIdx(pt.pos, octant->centroid)].push_back(pt);
            }

            for (int i = 0; i < 8; ++i) {
                if (child_buckets[i].empty()) continue;
                // Ensure child exists (using unique_ptr logic)
                if (!octant->children) octant->init_children(); 
                updateMultiRecursive(&octant->children[i], child_buckets[i], P_curr);
            }
        }
    }

    void updateRecursive(Octant* node, const PointCov& pt, const CovMat& P) {
        if (node->isLeaf()) {
            bool fused = false;
            for (auto& g : node->gaussians) {
                // Approximate normal using the smallest eigenvector of current covariance
                Eigen::SelfAdjointEigenSolver<CovMat> es(g.cov);
                if (g.checkFit(pt.pos, pt.cov)) {
                    g.fuseAdaptive(pt, P, es.eigenvectors().col(0));
                    fused = true; break;
                }
            }
            if (!fused) node->gaussians.emplace_back(pt);

            mergeGaussiansInOctant(node);

            if (node->gaussians.size() > max_gaussians_ && node->extent > min_extent_) split(node);
        } else {
            updateRecursive(&node->children[getIdx(pt.pos, node->centroid)], pt, P);
        }
    }

    void split(Octant* node, const CovMat& P_curr) {
        node->init_children();
        
        auto old_gaussians = std::move(node->gaussians);
        node->gaussians.clear();

        for (auto& g : old_gaussians) {
            // Re-wrap Gaussian into PointCov to re-insert
            PointCov tmp{g.mean, g.cov};
            // Note: We don't want to re-fuse and change R_map, just re-locate
            int idx = getIdx(g.mean, node->centroid);
            node->children[idx].gaussians.push_back(std::move(g));
        }
    }

    void mergeGaussiansInOctant(Octant* octant) {
        if (octant->gaussians.size() < 2) return;

        for (size_t i = 0; i < octant->gaussians.size(); ++i) {
            for (size_t j = i + 1; j < octant->gaussians.size(); ) {
                // Check if Gaussian J fits inside Gaussian I
                // We use the combined covariance S = Cov_i + Cov_j
                if (octant->gaussians[i].checkFit(octant->gaussians[j].mean, octant->gaussians[j].cov)) {
                    
                    // Perform the Weighted Bayesian Merge
                    octant->gaussians[i].merge(octant->gaussians[j]);
                    
                    // Remove the redundant Gaussian J
                    octant->gaussians.erase(octant->gaussians.begin() + j);
                } else {
                    ++j;
                }
            }
        }
    }

    static inline int getIdx(const Point& p, const Point& c) {
        return (p.x() > c.x() ? 1 : 0) | (p.y() > c.y() ? 2 : 0) | (p.z() > c.z() ? 4 : 0);
    }

    static bool boxOverlap(const Octant& node, const Point& q, double r2) {
        double d2 = 0;
        for(int i=0; i<3; ++i) {
            double min_b = node.centroid[i] - node.extent, max_b = node.centroid[i] + node.extent;
            if (q[i] < min_b) d2 += std::pow(min_b - q[i], 2);
            else if (q[i] > max_b) d2 += std::pow(q[i] - max_b, 2);
        }
        return d2 <= r2;
    }

    bool isQuerySphereInside(Octant* octant, const Point& q, double r2) const {
        // Distance from query to each face of the octant
        Point dists = octant->extent - (q - octant->centroid).cwiseAbs().array();
        
        // If query is outside or the sphere overlaps a boundary, return false
        if (dists.x() < 0 || dists.x() * dists.x() < r2) return false;
        if (dists.y() < 0 || dists.y() * dists.y() < r2) return false;
        if (dists.z() < 0 || dists.z() * dists.z() < r2) return false;
        
        return true;
    }

    bool knnRecursive(Octant* octant, const Point& q, NeighborHeap& heap) const {
        if (octant->isLeaf()) {
            for (const auto& g : octant->gaussians) {
                double d2 = (q - g.mean).squaredNorm();
                heap.add(g, d2);
            }
            return heap.full() && isQuerySphereInside(octant, q, heap.worstDist());
        }

        // 1. Visit the child containing the query point first
        int morton = getIdx(q, octant->centroid);
        if (octant->children[morton].centroid.size() > 0) { // Check if valid
             if (knnRecursive(&octant->children[morton], q, heap)) return true;
        }

        // 2. Visit other children in optimized order
        for (int i = 0; i < 7; ++i) {
            int c = ordered_indices[morton][i];
            Octant* child = &octant->children[c];

            // Pruning: skip if child cannot possibly contain a closer point
            if (heap.full() && !overlaps(child, q, heap.worstDist()))
                continue;

            if (knnRecursive(child, q, heap)) return true;
        }

        return heap.full() && isQuerySphereInside(octant, q, heap.worstDist());
    }

    static double minDistSq(Octant* node, const Point& q) {
        double d2 = 0;
        for(int i=0; i<3; ++i) {
            double min_b = node->centroid[i] - node->extent, max_b = node->centroid[i] + node->extent;
            if (q[i] < min_b) d2 += std::pow(min_b - q[i], 2);
            else if (q[i] > max_b) d2 += std::pow(q[i] - max_b, 2);
        }
        return d2;
    }

    static bool overlaps(Octant* node, const Point& q, double r2) {
        return minDistSq(node, q) <= r2;
    }
    
    // Friend class to allow IVox to access root_
    friend class GaussianIVox;
};

} // namespace gauss_mapping
} // namespace fast_limo