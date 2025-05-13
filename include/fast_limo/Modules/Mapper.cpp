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

// class fast_limo::Mapper
    // public

        Mapper::Mapper() : last_map_time(-1.), num_threads_(1), bb_init(false){
            this->map = KD_TREE<MapPoint>::Ptr (new KD_TREE<MapPoint>(0.3, 0.6, 0.2));

            // Init cfg values
            this->config.NUM_MATCH_POINTS = 5;
            this->config.MAX_NUM_PC2MATCH = 1.e+4;
            this->config.MAX_DIST_PLANE   = 2.0;
            this->config.PLANE_THRESHOLD  = 5.e-2;
            this->config.local_mapping    = true;

            this->config.ikdtree.cube_size = 300.0;
            this->config.ikdtree.rm_range  = 200.0;
        }

        void Mapper::set_num_threads(int n){
            if(n < 1) return;
            this->num_threads_ = n;
        }

        void Mapper::set_config(const Config::iKFoM::Mapping& cfg){
            this->config = cfg;
            map->Set_delete_criterion_param(config.ikdtree.delete_param);
            map->Set_balance_criterion_param(config.ikdtree.balance_param);
            map->set_downsample_param(config.ikdtree.voxel_size);
        }
                
        bool Mapper::exists(){
            return this->map->size() > 0;
        }

        int Mapper::size(){
            return this->map->size();
        }

        double Mapper::last_time(){
            return this->last_map_time;
        }

        BoxPointType Mapper::get_local_map(){
            return this->local_map_bb;
        }

        bool Mapper::is_relocated(){
            return this->relocated_;
        }

        Matches Mapper::match(State s, pcl::PointCloud<PointType>::Ptr& pc){

            if(not this->exists()) return matches;

            // if(not this->matches.empty()){ // we already found matches
            //     for (auto& match : matches) 
            //         match.update_global(s); // update global point
            //     return matches;
            // }

            int N0 = (pc->points.size() > config.MAX_NUM_PC2MATCH) ? pc->points.size() - config.MAX_NUM_PC2MATCH : 0;

            Matches init_matches;
            init_matches.resize(pc->points.size()-N0);

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size()-N0; i++){
                
                Eigen::Vector4f bl4_point(pc->points[i].x, pc->points[i].y, pc->points[i].z, 1.); // base link 4d point
                Eigen::Vector4f global_point = s.get_RT() * bl4_point;                            // global 4d point == [x', y', z', 1.0]
                Match match = this->match_plane(global_point, bl4_point);                         // point-to-plane match

                init_matches[i] = match; 
            }

            Matches chosen_matches;
            for(int j = 0; j < init_matches.size(); j++){
                if(init_matches[j].lisanAlGaib())
                    chosen_matches.push_back(init_matches[j]); // if match is chosen, push it
            }

            this->matches = chosen_matches; // save matches for next iter
            return chosen_matches;
        }
        
        void Mapper::add(pcl::PointCloud<PointType>::Ptr& pc, State& s, double time, bool downsample){

            // Initialize local map dimensions 
            if(not this->bb_init)
                this->set_bb_dim(s);
            
            // Move local map (if necessary)
            this->fov_segment(s);

            // Add new points to the map
            this->add(pc, time, downsample);

            this->last_map_time = time;
        }

        void Mapper::add(pcl::PointCloud<PointType>::Ptr& pc, double time, bool downsample){
            if(pc->points.size() < 1) return;

            // If map doesn't exists, build one
            if(not this->exists()) this->build(pc);
            else this->add_pointcloud(pc, downsample);

            this->last_map_time = time;
        }

        void Mapper::load_map(pcl::PointCloud<PointType>::Ptr& full_map){

            KD_TREE<MapPoint>::Ptr new_map = KD_TREE<MapPoint>::Ptr (new KD_TREE<MapPoint>(0.3, 0.6, 0.01));            
            MapPoints map_vec;
            map_vec.resize(full_map->points.size());
            
            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < full_map->points.size(); i++)
                map_vec[i] = MapPoint(full_map->points[i].x, 
                                    full_map->points[i].y, 
                                    full_map->points[i].z);

            new_map->Build(map_vec);
            this->relocated_ = true;
            this->map = new_map;
        }

        bool Mapper::get_map(pcl::PointCloud<PointType>::Ptr& pc){
            
            if(not this->exists()) return false;
            this->get_full_map(pc);
            return pc->points.size() > 0;

        }

    // private

        void Mapper::build(pcl::PointCloud<PointType>::Ptr& pc){
            MapPoints map_vec;
            map_vec.resize(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++)
                map_vec[i] = MapPoint( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Build(map_vec);
        }

        void Mapper::get_full_map(pcl::PointCloud<PointType>::Ptr& pc){
            
            MapPoints map_vec;
            map_vec.reserve(this->map->size());
            this->map->flatten(this->map->Root_Node, map_vec, NOT_RECORD);
                        
            pc->points.resize(map_vec.size());
            for(int i = 0; i < map_vec.size(); i++){
                pc->points[i].x = map_vec[i].x;
                pc->points[i].y = map_vec[i].y;
                pc->points[i].z = map_vec[i].z;
            }
            
            pc->width = pc->points.size();
            pc->height = 1;

        }

        void Mapper::add_pointcloud(pcl::PointCloud<PointType>::Ptr& pc, bool downsample){
            MapPoints map_vec;
            map_vec.resize(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size(); i++)
                map_vec[i] = MapPoint( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Add_Points(map_vec, downsample);
        }

        Match Mapper::match_plane(Eigen::Vector4f& p, Eigen::Vector4f& p_local) {

            // Find k nearest points
            MapPoints near_points;
            std::vector<float> pointSearchSqDis(this->config.NUM_MATCH_POINTS);
            this->map->Nearest_Search(MapPoint(p(0), p(1), p(2)), this->config.NUM_MATCH_POINTS, near_points, pointSearchSqDis);

            // Construct a plane fitting between them
            return Match( p.head(3), p_local.head(3), Plane (near_points, pointSearchSqDis, &config) );
        }

        void Mapper::set_bb_dim(State& s){

            if(not this->exists()) return; // wait until the map is build

            this->mtx_local_map.lock();
            for(int k=0; k < 3; k++){
                this->local_map_bb.vertex_min[k] = s.p(k) - this->config.ikdtree.cube_size/2.0;
                this->local_map_bb.vertex_max[k] = s.p(k) + this->config.ikdtree.cube_size/2.0;
            }
            this->mtx_local_map.unlock();
            this->bb_init = true;
        }

        void Mapper::fov_segment(State& s){

            if(not this->bb_init) return;

            std::vector<BoxPointType> cube2rm; // 3D box (cube) to remove from map

            double dist2edge[3][2];
            bool need2move = false;
            for(unsigned int i=0; i < 3; i++){
                dist2edge[i][0] = fabs(s.p(i) - local_map_bb.vertex_min[i]);
                dist2edge[i][1] = fabs(s.p(i) - local_map_bb.vertex_max[i]);
                if(dist2edge[i][0] < this->config.ikdtree.rm_range || dist2edge[i][1] < this->config.ikdtree.rm_range) need2move = true;
            }

            if(!need2move) return;

            BoxPointType new_local_bb = this->local_map_bb;
            BoxPointType tmp_local_bb;
            double move_dist = 0.5*this->config.ikdtree.cube_size;
            for(unsigned int k=0; k < 3; k++){
                tmp_local_bb = this->local_map_bb;
                if(dist2edge[k][0] < this->config.ikdtree.rm_range){
                    new_local_bb.vertex_max[k] -= move_dist;
                    new_local_bb.vertex_min[k] -= move_dist;
                    tmp_local_bb.vertex_min[k] = this->local_map_bb.vertex_max[k] - move_dist;
                    cube2rm.push_back(tmp_local_bb);
                }else if(dist2edge[k][1] < this->config.ikdtree.rm_range){
                    new_local_bb.vertex_max[k] += move_dist;
                    new_local_bb.vertex_min[k] += move_dist;
                    tmp_local_bb.vertex_max[k] = this->local_map_bb.vertex_min[k] + move_dist;
                    cube2rm.push_back(tmp_local_bb);
                }
            }

            this->mtx_local_map.lock(); // lock local map
            this->local_map_bb = new_local_bb;
            this->mtx_local_map.unlock(); // unlock after finished writing local map

            if(cube2rm.size() > 0)
                this->map->Delete_Point_Boxes(cube2rm);
        }
