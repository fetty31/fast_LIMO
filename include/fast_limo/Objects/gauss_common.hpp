#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace fast_limo {
namespace gauss_mapping {

using Point = Eigen::Vector3d;
using CovMat = Eigen::Matrix3d;

struct PointCov {
    Point pos;
    CovMat cov; 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using VecPointCov = std::vector<PointCov, Eigen::aligned_allocator<PointCov>>;

struct Gaussian {
    Point mean;
    CovMat cov;
    CovMat R_map; // AKF: Adaptive Measurement Noise Covariance
    int count = 0;
    static constexpr double chi_threshold = 7.815;

    Gaussian(const PointCov& pt) : mean(pt.pos), cov(pt.cov), R_map(pt.cov), count(1) {}

    bool checkFit(const Point& p_pos, const CovMat& p_cov) const {
        Point diff = p_pos - mean;

        // Total uncertainty = Geometric spread + Adaptive measurement noise + New point noise
        // In AKF-LIO, R_map already 'absorbs' the historical point noise.
        Eigen::Matrix3d S = cov + R_map + p_cov; 
        
        double mahalanobis_dist = diff.transpose() * S.ldlt().solve(diff);
        
        return mahalanobis_dist < chi_threshold;
    }
    

    // Adaptive Fusion from AKF-LIO paper
    void fuseAdaptive(const PointCov& pt, const CovMat& P_curr, const Point& normal) {
        double residual = normal.transpose() * (pt.pos - mean);
        double HPH = normal.transpose() * P_curr * normal;
        double R_obs_val = residual * residual + HPH;
        CovMat R_obs = CovMat::Identity() * R_obs_val;

        double alpha = static_cast<double>(count) / (count + 1);
        R_map = alpha * R_map + (1.0 - alpha) * R_obs;

        Eigen::Matrix3d S = cov + R_map;
        Eigen::Matrix3d K = (S.ldlt().solve(cov)).transpose();
        
        mean += K * (pt.pos - mean);
        cov = (Eigen::Matrix3d::Identity() - K) * cov;
        cov = 0.5 * (cov + cov.transpose().eval()); // Keep symmetric
        count++;
    }

    void merge(const Gaussian& other) {
        double n1 = static_cast<double>(this->count);
        double n2 = static_cast<double>(other.count);
        double n_total = n1 + n2;

        // Save old mean for covariance calculation
        Point mu1 = this->mean;
        Point mu2 = other.mean;

        // Update Mean
        this->mean = (n1 * mu1 + n2 * mu2) / n_total;

        // Update Covariance using the Parallel Axis Theorem logic
        CovMat term1 = n1 * (this->cov + mu1 * mu1.transpose());
        CovMat term2 = n2 * (other.cov + mu2 * mu2.transpose());
        this->cov = (term1 + term2) / n_total - (this->mean * this->mean.transpose());
        
        // Update Adaptive Noise (AKF-LIO specific)
        this->R_map = (n1 * this->R_map + n2 * other.R_map) / n_total;

        this->count = static_cast<int>(n_total);
        // Re-symmetrize to prevent drift
        this->cov = 0.5 * (this->cov + this->cov.transpose().eval());
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using GaussianVector = std::vector<Gaussian, Eigen::aligned_allocator<Gaussian>>;

struct Octant {
    Point centroid;
    double extent;
    GaussianVector gaussians;
    std::unique_ptr<Octant[]> children; // Contiguous memory for 8 children

    Octant(const Point& c, double e) : centroid(c), extent(e), children(nullptr) { }

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

struct Neighbor {
    double dist_sq;
    Gaussian g;
    // Max-heap comparison
    bool operator<(const Neighbor& other) const { return dist_sq < other.dist_sq; }
};

class NeighborHeap {
public:
    explicit NeighborHeap(size_t k) : k_(k) { data_.reserve(k); }

    void add(const Gaussian& g, double d2) {
        if (data_.size() < k_) {
            data_.push_back({d2, g});
            std::push_heap(data_.begin(), data_.end());
        } else if (d2 < data_.front().dist_sq) {
            std::pop_heap(data_.begin(), data_.end());
            data_.back() = {d2, g};
            std::push_heap(data_.begin(), data_.end());
        }
    }

    bool full() const { return data_.size() >= k_; }
    double worstDist() const { return data_.empty() ? std::numeric_limits<double>::max() : data_.front().dist_sq; }
    const std::vector<Neighbor>& data() const { return data_; }

private:
    size_t k_;
    std::vector<Neighbor> data_;
};

} // namespace gauss_mapping
} // namespace fast_limo