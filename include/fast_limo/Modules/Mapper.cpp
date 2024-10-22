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

#include "fast_limo/Modules/Mapper.hpp"



IKDTree::IKDTree() : last_map_time_(-1.),
                   num_threads_(1) {

  map = KD_TREE<MapPoint>::Ptr(new KD_TREE<MapPoint>(0.3, 0.6, 0.2));

  // Init cfg values
  config.NUM_MATCH_POINTS = 5;
  config.MAX_NUM_PC2MATCH = 1.e+4;
  config.MAX_DIST_PLANE   = 2.0;
  config.PLANE_THRESHOLD  = 5.e-2;
  config.local_mapping    = true;

  config.ikdtree.cube_size = 300.0;
  config.ikdtree.rm_range  = 200.0;
}
    
bool IKDTree::exists() {
  return map->size() > 0;
}

int IKDTree::size() {
  return map->size();
}

double IKDTree::last_time() {
  return last_map_time_;
}


void IKDTree::add(PointCloudT::Ptr& pc, double time, bool downsample) {
  if (pc->points.size() < 1)
    return;

  // If map doesn't exists, build one
  if (not exists()) {
    build(pc);
  } else {
    MapPoints map_vec;
    map_vec.reserve(pc->points.size());

    for (int i = 0; i < pc->points.size(); i++)
      map_vec.emplace_back(pc->points[i].x, pc->points[i].y, pc->points[i].z);

    map->Add_Points(map_vec, downsample);
  }

  last_map_time_ = time;
}

// private

void IKDTree::build(PointCloudT::Ptr& pc) {
  MapPoints map_vec;
  map_vec.reserve(pc->points.size());

  for(int i = 0; i < pc->points.size (); i++)
    map_vec.emplace_back(pc->points[i].x, pc->points[i].y, pc->points[i].z);

  this->map->Build(map_vec);
}

void IKDTree::knn(const MapPoint& p,
         int& k,
         MapPoints& near_points,
         std::vector<float>& sqDist) {

  map->Nearest_Search(p, k, near_points, sqDist);
}









Octree::Octree() : last_map_time_(-1.),
                   num_threads_(1) {
  
  map.set_order(false);
  map.set_min_extent(5.); // float
  map.set_bucket_size(5); // size_t
  map.set_down_size(true);  // bool

}
    
bool Octree::exists() {
  return size() > 0;
}

int Octree::size() {
  return (int)map.size();
}

double Octree::last_time() {
  return last_map_time_;
}


void Octree::add(PointCloudT::Ptr& pc, double time, bool downsample) {
  if (pc->points.size() < 1)
    return;

  // If map doesn't exists, build one
  if (not exists()) {
    build(pc);
  } else {
    MapPoints map_vec;
    map_vec.reserve(pc->points.size());

    for (int i = 0; i < pc->points.size(); i++)
      map_vec.emplace_back(pc->points[i].x, pc->points[i].y, pc->points[i].z);

    map.update(map_vec, downsample);
  }

  last_map_time_ = time;
}

// private

void Octree::build(PointCloudT::Ptr& pc) {
  MapPoints map_vec;
  map_vec.reserve(pc->points.size());

  for(int i = 0; i < pc->points.size (); i++)
    map_vec.emplace_back(pc->points[i].x, pc->points[i].y, pc->points[i].z);

  map.initialize(map_vec);
}

void Octree::knn(const MapPoint& p,
         int& k,
         MapPoints& near_points,
         std::vector<float>& sqDist) {
  map.knnNeighbors<MapPoint>(p, k, near_points, sqDist);
}