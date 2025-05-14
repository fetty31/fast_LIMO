#pragma once

// Copyright (c) 2023 Jun Zhu, Tsinghua University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRES S OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include <algorithm>
#include <utility>
#include <chrono>
#include <fstream>
#include <functional>
#include <Eigen/Dense>

namespace fast_limo {

namespace octree {

using Point  = Eigen::Vector3f;
using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

struct Heap {
  struct DistancePoint {
    float dist;
    Point point;

    DistancePoint() : dist(std::numeric_limits<float>::max()) { }
  };

  size_t capacity;
  size_t count;
  float worst_distance_;
  std::vector<DistancePoint> data;

  Heap(size_t capacity_) : capacity(capacity_), count(0), data(capacity_) { }

  bool full() const {
    return count == capacity;
  }

  std::vector<DistancePoint> get_data() {
    return std::vector<DistancePoint>(data.begin(), data.begin()+count);
  }

  float worstDist() {
    return full() ? data[count-1].dist : std::numeric_limits<float>::max();
  }

  void addPoint(const Point& p, float dist) {
    if (full() and dist >= data[count-1].dist)
      return;

    if (count < capacity)
      ++count;

    int i = static_cast<int>(count) - 1;
    while (i > 0 && data[i - 1].dist > dist) {
      data[i] = data[i - 1];
      --i;
    }

    data[i].dist = dist;
    data[i].point = p;
  }
};


/*
 * Octant is a node in the octree. It contains a list of points and
 * pointers to its children.
 * 
 * The octant is a cube with a centroid and an extent. The centroid
 * is the center of the cube, and the extent is half of the side
 * length of the cube. The octant is divided into 8 children,
 * each of which is a smaller cube. The children are created
 * when the number of points in the octant exceeds a certain
 * threshold (bucket_size_). The children are created
 * by splitting the octant along its axes.
 */
struct Octant {
  bool  is_active;   // whether the octant is active
  Point centroid;    // center of the octant
  float extent;      // half of side-length

  Points points;     // collection of points that fall into this octant (used when the octant is a leaf)
  Octant **child;    // pointer to an array of 8 pointers (one for each sub-octant)

  Octant() : is_active(true), extent(0.f), child(nullptr) { }

  ~Octant() {
    if (child != nullptr) {
      for (int i = 0; i < 8; ++i) {
        if (child[i] != nullptr)
          delete child[i];
      }

      delete[] child;
      child = nullptr;
    }

    points.clear();
  }

  // Allocates an array for 8 child pointers and initializes them to `nullptr`
  void init_child() {
    child = new Octant*[8]();
  }

};


struct Octree {

  Octant *root_;        // pointer to the root `Octant` of the tree
  size_t num_points_;   // number of points in the tree
  size_t bucket_size_;  // maximum number of points allowed in an octant before it gets subdivided
  float min_extent_;    // minimum extent of the octant (used to stop subdividing)
  bool downsample_;     // flag indicating if downsampling should occur under certain conditions

  // used in the kNN search to order the traversal of child octants for efficiency by checking the most promising child first
  size_t ordered_indices[8][7] = {
    {1, 2, 4, 3, 5, 6, 7},
    {0, 3, 5, 2, 4, 7, 6},
    {0, 3, 6, 1, 4, 7, 5},
    {1, 2, 7, 0, 5, 6, 4},
    {0, 5, 6, 1, 2, 7, 3},
    {1, 4, 7, 0, 3, 6, 2},
    {2, 4, 7, 0, 3, 5, 1},
    {3, 5, 6, 1, 2, 4, 0}
  };

  Octree() : bucket_size_(32), 
             min_extent_(0.2f), 
             downsample_(true),
             root_(nullptr), 
             num_points_(0) { }


  Octree(size_t bucketSize_, 
         bool downsample_, 
         float minExtent_) : bucket_size_(bucketSize_), 
                             min_extent_(minExtent_), 
                             downsample_(downsample_),
                             root_(nullptr), 
                             num_points_(0) { }

  ~Octree() {
    clear();
  }

  void setMinExtent(float extent) {
    min_extent_ = extent;
  }

  void setBucketSize(size_t bucket_size_) {
    bucket_size_ = bucket_size_;
  }

  void setDownsample(bool down_size) {
    downsample_ = down_size;
  }

  void clear() {
    delete root_;
    root_ = nullptr;
  }

  size_t size() {
    return num_points_;
  }


  // Idea paralelitzar donant-li a cada thread un octant diferent 
  // i finalment concatenar els vectors de cada thread
  template <typename PointT, typename ContainerT>
  ContainerT getData() {
    Points points;
    get_points(root_, points);

    ContainerT out;
    out.reserve(points.size());

    for (const auto& p : points) {
      PointT pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      out.push_back(pt);
    }

    return out;
  }

  void get_points(const Octant* octant, Points& points) {
    if (octant == nullptr)
      return;

    if (octant->child == nullptr) {
      points.insert(points.end(), octant->points.begin(), octant->points.end());
    } else {
      for (int i = 0; i < 8; i++) { 
        get_points(octant->child[i], points);
      }
    }
  }

  /* Process the input points to remove NaN values and calculate the bounding box
   * of the points. The bounding box is defined by the minimum and maximum points.
   * The function returns a vector of valid points.
   */
  template <typename ContainerT>
  Points processPoints(const ContainerT& pts, Point& min, Point& max) {
  
    Points out;
    out.resize(pts->size(), Point()); // changed from pts.size() to pts->size()

    size_t n(0);
    for (const auto& pt : *pts) { // changed from pts to *pts

      if (std::isnan(pt.x) or std::isnan(pt.y) or std::isnan(pt.z))
        continue;

      out[n] = Point(pt.x, pt.y, pt.z);
      n++;

      if (n == 0) {
        min.x() = max.x() = pt.x;
        min.y() = max.y() = pt.y;
        min.z() = max.z() = pt.z;

      } else {
        min.x() = pt.x < min.x() ? pt.x : min.x();
        min.y() = pt.y < min.y() ? pt.y : min.y();
        min.z() = pt.z < min.z() ? pt.z : min.z();

        max.x() = pt.x > max.x() ? pt.x : max.x();
        max.y() = pt.y > max.y() ? pt.y : max.y();
        max.z() = pt.z > max.z() ? pt.z : max.z();
      }
    }

    out.resize(n);
    return out;
  }

  inline size_t mortonCode(const Point& p, const Point& centroid) {
    size_t out(0);
    if (p.x() > centroid.x()) out |= 1;
    if (p.y() > centroid.y()) out |= 2;
    if (p.z() > centroid.z()) out |= 4;
    return out;
  }

  /* * Initialize the octree with the input points. The function first clears
   * the existing tree and then processes the input points to create a new
   * octree. The bounding box of the points is calculated and used to create
   * the root octant.
   */
  template <typename ContainerT>
  void initialize(ContainerT& pts) {

    clear();
    
    Point min = Point::Constant(std::numeric_limits<float>::max());
    Point max = Point::Constant(std::numeric_limits<float>::lowest());
    Points points = processPoints(pts, min, max);

    if (points.empty())
      return;

    Point extent = 0.5f * (max - min);
    Point centroid = min + extent;

    root_ = createOctant(centroid, extent.maxCoeff(), points);
  }

  
  Octant* createOctant(const Point& centroid, 
                       float maxextent, 
                       const Points& points) {

    Octant* octant = new Octant;
    octant->centroid = centroid;
    octant->extent = maxextent;

    static const float factor[] = {-0.5f, 0.5f};
    if (points.size() > bucket_size_ && maxextent > 2*min_extent_) {
      octant->init_child();

      std::vector<Points> child_points(8, Points());
      for (const auto& p : points) {
        child_points[mortonCode(p, centroid)].push_back(std::move(p));
      }

      for (int i=0; i<8; ++i) {
        if (child_points[i].empty())
          continue;
        
        Point child_centroid = Point(
          centroid.x() + factor[(i & 1) > 0] * maxextent,
          centroid.y() + factor[(i & 2) > 0] * maxextent,
          centroid.z() + factor[(i & 4) > 0] * maxextent);
        
        octant->child[i] = createOctant(child_centroid, 
                                        maxextent*0.5f, 
                                        std::move(child_points[i]));
      }

    } else {
      num_points_ += points.size();
      octant->points = std::move(points);
    }

    return octant;
  }


  template <typename ContainerT>
  void update(ContainerT& pts) {

    if (root_ == nullptr) {
      initialize(pts);
      return;
    }
    
    Point min = Point::Constant(std::numeric_limits<float>::max());
    Point max = Point::Constant(std::numeric_limits<float>::lowest());
    Points points = processPoints(pts, min, max);

    static const float factor[] = {-0.5f, 0.5f};
    auto expandTree = [&](const Point& boundary) {
      while ((boundary - root_->centroid).cwiseAbs().maxCoeff() > root_->extent) {
        float parent_extent = 2 * root_->extent;
        Point parent_centroid(
          root_->centroid.x() + factor[boundary.x() > root_->centroid.x()] * parent_extent,
          root_->centroid.y() + factor[boundary.y() > root_->centroid.y()] * parent_extent,
          root_->centroid.z() + factor[boundary.z() > root_->centroid.z()] * parent_extent);
      
        Octant* octant = new Octant;
        octant->centroid = parent_centroid;
        octant->extent = parent_extent;
      
        octant->init_child();
        octant->child[mortonCode(root_->centroid, parent_centroid)] = root_;

        root_ = octant;
      }
    };

    expandTree(max);
    expandTree(min);

    updateOctant(root_, points);
  }


  void updateOctant(Octant*& octant, const Points& points) {

    static const float factor[] = {-0.5f, 0.5f};
    if (octant->child == nullptr) {

      if (octant->points.size()+points.size() > bucket_size_ and 
          octant->extent > 2*min_extent_) {
            
        num_points_ -= octant->points.size();

        octant->points.insert(octant->points.end(), points.begin(), points.end());
        Octant* newOctant = createOctant(octant->centroid, 
                                         octant->extent, 
                                         octant->points);
        delete octant;
        octant = newOctant;

      } else {

        if (downsample_ and 
            octant->extent <= 2*min_extent_ and 
            octant->points.size() > bucket_size_/8) { return; }
        
        octant->points.insert(octant->points.end(), points.begin(), points.end());
        num_points_ += points.size();
      }

    } else {
      std::vector<Points> child_points(8, Points());
      for (const auto& p : points) {
        child_points[mortonCode(p, octant->centroid)].push_back(std::move(p));
      }

      for (size_t i=0; i<8; i++) {

        if (child_points[i].empty())
          continue; 
         
        if (octant->child[i] == nullptr) {
          Point child_centroid = Point(
            octant->centroid.x() + factor[(i & 1) > 0] * octant->extent,
            octant->centroid.y() + factor[(i & 2) > 0] * octant->extent,
            octant->centroid.z() + factor[(i & 4) > 0] * octant->extent);
            
          octant->child[i] = createOctant(child_centroid, 
                                          octant->extent*0.5f,
                                          std::move(child_points[i]));
        } else {
          updateOctant(octant->child[i], std::move(child_points[i]));
        }
      }
    }
  }


  bool overlaps(const Octant* octant, const Point& query, const float& sqr_radius) {

    Point dist = ((query - octant->centroid).cwiseAbs().array() - octant->extent);
    
    if ((dist.x() > 0 and dist.x()*dist.x() > sqr_radius) or
        (dist.y() > 0 and dist.y()*dist.y() > sqr_radius) or
        (dist.z() > 0 and dist.z()*dist.z() > sqr_radius))
      return false;

    int num_less_extent = (dist.x() < 0) + (dist.y() < 0) + (dist.z() < 0);

    if (num_less_extent > 1)
      return true;

    return (dist.cwiseMax(0.f).squaredNorm() < sqr_radius);
  };


  template <typename PointT>
  void radiusSearch(const PointT& query, 
                       float radius, 
                       std::vector<PointT>& neighbors, 
                       std::vector<float>& distances) {

    if (root_ == nullptr)
      return;

    neighbors.clear();
    distances.clear();

    Points points;
    Point q = Point(query.x, query.y, query.z);
    radiusSearch(root_, q, radius*radius, points, distances);

    neighbors.reserve(points.size());
    for (const auto& p : points) {
      PointT pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      neighbors.push_back(pt);
    }
  }

   void radiusSearch(const Octant* octant, 
                        const Point& query, 
                        float sqr_radius, 
                        Points& neighbors, 
                        std::vector<float> &distances) {

    auto contains = [&]() {
      return (
        (query - octant->centroid).cwiseAbs().array() + octant->extent
      ).array().square().sum() < sqr_radius;
    };

    if (3*octant->extent*octant->extent < sqr_radius and contains()) {
        
      Points points;
      get_points(octant, points);

      for (const auto& p : points) {
        float sqr_dist = (p - query).squaredNorm();
        distances.push_back(sqr_dist);
        neighbors.push_back(p);
      }

      return;
    }

    if (octant->child == nullptr) {
      for (const auto& p : octant->points) {
        float sqr_dist = (p - query).squaredNorm();
        if (sqr_dist < sqr_radius) {
          distances.push_back(sqr_dist);
          neighbors.push_back(p);
        }
      }

      return;
    }

    for (size_t c=0; c<8; c++) {
      if (octant->child[c] == nullptr or !overlaps(octant->child[c], query, sqr_radius))
        continue;

      radiusSearch(octant->child[c], query, sqr_radius, neighbors, distances);
    }
  }


  template <typename PointT>
  void knn(const PointT& query, 
           int k, 
           std::vector<PointT> &neighbors, 
           std::vector<float> &distances) {
    
    if (root_ == nullptr)
      return;
    
    neighbors.clear();
    distances.clear();
      
    Heap heap(k);
    Point q = Point(query.x, query.y, query.z);
    knn(root_, q, heap);
    
    std::vector<Heap::DistancePoint> points = heap.get_data();

    neighbors.reserve(points.size());
    distances.reserve(points.size());

    for (auto& p_dist : points) {
      PointT pt;
      pt.x = p_dist.point.x();
      pt.y = p_dist.point.y();
      pt.z = p_dist.point.z();
      neighbors.push_back(pt);
      distances.push_back(p_dist.dist);
    }
  }


  bool knn(const Octant* octant, Point& query, Heap& heap) {

    auto inside = [&](const float& radius) {
      Point dist = octant->extent - (query - octant->centroid).cwiseAbs().array();
    
      return (dist.x() < 0 or dist.x()*dist.x() < radius) ? false : 
             (dist.y() < 0 or dist.y()*dist.y() < radius) ? false :
             (dist.z() < 0 or dist.z()*dist.z() < radius) ? false :
             true;
    };

    if (octant->child == nullptr) {

      for (const auto& p : octant->points) {
        float sqr_dist = (query - p).squaredNorm();
        heap.addPoint(p, sqr_dist);
      }

      return heap.full() and inside(heap.worstDist());
    }

    size_t morton = mortonCode(query, octant->centroid);
    if (octant->child[morton] != nullptr) {
      if (knn(octant->child[morton], query, heap))
        return true;
    }

    for (int i=0; i<7; i++) {
      int c = ordered_indices[morton][i];

      if (octant->child[c] == nullptr)
        continue;

      if (heap.full() and !overlaps(octant->child[c], query, heap.worstDist()))
        continue;

      if (knn(octant->child[c], query, heap))
        return true;
    }

    return heap.full() and inside(heap.worstDist());
  }

};

} // namespace dummy

} // namespace fast_limo

