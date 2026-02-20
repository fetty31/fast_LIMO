#pragma once

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

namespace fast_limo {
namespace gauss_octree {

using Point = Eigen::Vector3d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

// Represents a single 3D Gaussian distribution
struct Gaussian {
    Point mean;
    Eigen::Matrix3d cov;
    Eigen::Matrix3d cov_inv;
    int count;
    double chi_threshold = 7.815f; // p < 0.05 for 3 DOF

    Gaussian(const Point& p) {
        mean = p;
        // Initialize with a small isotropic prior to prevent singularity
        cov = Eigen::Matrix3d::Identity() * 1e-4; 
        count = 1;
        updateInverse();
    }

    void updateInverse() {
        // Add small epsilon to diagonal for numerical stability
        cov_inv = (cov + Eigen::Matrix3d::Identity() * 1e-6).inverse();
    }

    // Check if a point belongs to this distribution using Mahalanobis Distance
    bool checkFit(const Point& p) const {
        Point diff = p - mean;
        double dist_sq = diff.transpose() * cov_inv * diff;
        return dist_sq < chi_threshold;
    }

    // Incremental update of Mean and Covariance
    void fuse(const Point& p) {
        Point pt = p;
        count++;
        Point delta = pt - mean;
        mean += delta / count;
        Point delta2 = pt - mean;
        cov += delta * delta2.transpose();
        updateInverse();
    }

    // Merge another Gaussian into this one
    void merge(const Gaussian& other) {
        double n1 = static_cast<double>(this->count);
        double n2 = static_cast<double>(other.count);
        double n_total = n1 + n2;

        Point combined_mean = (n1 * this->mean + n2 * other.mean) / n_total;

        Eigen::Matrix3d term1 = n1 * (this->cov + (this->mean - combined_mean) * (this->mean - combined_mean).transpose());
        Eigen::Matrix3d term2 = n2 * (other.cov + (other.mean - combined_mean) * (other.mean - combined_mean).transpose());
        
        this->mean = combined_mean;
        this->cov = (term1 + term2) / n_total;
        this->count = static_cast<int>(n_total);
        this->updateInverse();
    }
};

struct Octant {
    Point centroid;
    double extent;
    std::vector<Gaussian> gaussians;
    Octant** child;

    Octant() : extent(0.f), child(nullptr) {}

    ~Octant() {
        if (child != nullptr) {
            for (int i = 0; i < 8; ++i) {
                if (child[i] != nullptr) delete child[i];
            }
            delete[] child;
        }
    }

    void init_child() {
        child = new Octant*[8]();
    }
};

class Octree {
public:
    Octant* root_;
    size_t max_gaussians_per_leaf_;
    double min_extent_;

    Octree(size_t max_gaussians = 5, double min_extent = 0.2f)
        : root_(nullptr), max_gaussians_per_leaf_(max_gaussians), min_extent_(min_extent) {}

    ~Octree() { clear(); }

    void clear() {
        delete root_;
        root_ = nullptr;
    }

    // Entry point for adding points
    void update(const Points& pts) {
        if (pts.empty()) return;

        if (root_ == nullptr) {
            initialize(pts);
        } else {
            // Check if points are outside current root bounds
            expandRootIfNeeded(pts);
            updateOctant(root_, pts);
        }
    }

private:
    void initialize(const Points& pts) {
        Point min = Point::Constant(std::numeric_limits<double>::max());
        Point max = Point::Constant(std::numeric_limits<double>::lowest());
        for (const auto& p : pts) {
            min = min.cwiseMin(p);
            max = max.cwiseMax(p);
        }
        Point extent_vec = 0.5f * (max - min);
        double max_extent = extent_vec.maxCoeff();
        Point centroid = min + Point::Constant(max_extent);

        root_ = new Octant();
        root_->centroid = centroid;
        root_->extent = max_extent;
        updateOctant(root_, pts);
    }

    void expandRootIfNeeded(const Points& pts) {
        static const double factor[] = {-1.0f, 1.0f};
        for (const auto& p : pts) {
            while ((p - root_->centroid).cwiseAbs().maxCoeff() > root_->extent) {
                double parent_extent = 2.0f * root_->extent;
                Point parent_centroid = root_->centroid;
                for(int i=0; i<3; ++i) {
                    parent_centroid[i] += (p[i] > root_->centroid[i] ? 1.0f : -1.0f) * root_->extent;
                }

                Octant* new_root = new Octant();
                new_root->centroid = parent_centroid;
                new_root->extent = parent_extent;
                new_root->init_child();
                new_root->child[mortonCode(root_->centroid, parent_centroid)] = root_;
                root_ = new_root;
            }
        }
    }

    void updateOctant(Octant*& octant, const Points& points) {
        if (octant->child == nullptr) {
            // Leaf Node logic
            for (const auto& p : points) {
                bool fused = false;
                for (auto& g : octant->gaussians) {
                    if (g.checkFit(p)) {
                        g.fuse(p);
                        fused = true;
                        break;
                    }
                }
                if (!fused) {
                    octant->gaussians.emplace_back(p);
                }
            }

            // Cleanup: Merge overlapping Gaussians
            mergeGaussiansInOctant(octant);

            // Split if too many distinct models
            if (octant->gaussians.size() > max_gaussians_per_leaf_ && octant->extent > 2 * min_extent_) {
                splitOctant(octant);
            }
        } else {
            // Internal Node logic: Distribute points to children
            std::vector<Points> child_points(8);
            for (const auto& p : points) {
                child_points[mortonCode(p, octant->centroid)].push_back(p);
            }

            for (int i = 0; i < 8; ++i) {
                if (child_points[i].empty()) continue;
                if (octant->child[i] == nullptr) {
                    octant->child[i] = createChildNode(octant, i);
                }
                updateOctant(octant->child[i], child_points[i]);
            }
        }
    }

    void mergeGaussiansInOctant(Octant* octant) {
        if (octant->gaussians.size() < 2) return;
        for (size_t i = 0; i < octant->gaussians.size(); ++i) {
            for (size_t j = i + 1; j < octant->gaussians.size(); ) {
                if (octant->gaussians[i].checkFit(octant->gaussians[j].mean)) {
                    octant->gaussians[i].merge(octant->gaussians[j]);
                    octant->gaussians.erase(octant->gaussians.begin() + j);
                } else {
                    ++j;
                }
            }
        }
    }

    void splitOctant(Octant* octant) {
        octant->init_child();
        for (auto& g : octant->gaussians) {
            size_t idx = mortonCode(g.mean, octant->centroid);
            if (octant->child[idx] == nullptr) {
                octant->child[idx] = createChildNode(octant, idx);
            }
            octant->child[idx]->gaussians.push_back(std::move(g));
        }
        octant->gaussians.clear();
    }

    Octant* createChildNode(Octant* parent, int index) {
        static const double f[] = {-0.5f, 0.5f};
        Octant* child = new Octant();
        child->extent = parent->extent * 0.5f;
        child->centroid = Point(
            parent->centroid(0) + f[(index & 1) > 0] * parent->extent,
            parent->centroid(1) + f[(index & 2) > 0] * parent->extent,
            parent->centroid(2) + f[(index & 4) > 0] * parent->extent
        );
        return child;
    }

    inline size_t mortonCode(const Point& p, const Point& centroid) {
        size_t out(0);
        if (p(0) > centroid(0)) out |= 1;
        if (p(1) > centroid(1)) out |= 2;
        if (p(2) > centroid(2)) out |= 4;
        return out;
    }
};

} // namespace gauss_octree
} // namespace fast_limo