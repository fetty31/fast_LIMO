#include "fast_limo/Modules/Mapper.hpp"

// class fast_limo::Mapper
    // public

        Mapper::Mapper() : last_map_time(-1.), num_threads_(1){
            this->map = KD_TREE<MapPoint>::Ptr (new KD_TREE<MapPoint>(0., 0., 0.));
        }

        void Mapper::set_num_threads(int n){
            if(n < 1) return;
            this->num_threads_ = n;
        }
                
        bool Mapper::exists(){
            return this->map->size() > 1;
        }

        int Mapper::size(){
            return this->map->size();
        }

        double Mapper::last_time(){
            return this->last_map_time;
        }

        Matches Mapper::match(State s, pcl::PointCloud<PointType>::ConstPtr& pc){
            Matches matches;
            if(not this->exists()) return matches;
            matches.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++){
                
                Eigen::Vector4f bl_point(pc->points[i].x, pc->points[i].y, pc->points[i].z, 1.); // base link 4d point
                Eigen::Vector4f global_point = s.get_RT() * bl_point;                               // global 4d point == [x', y', z', 1.0]
                Match match = this->match_plane(global_point);                                      // point-to-plane match

                if(match.lisanAlGaib()) matches.push_back(match); // if match is chosen, push it
            }

            return matches;
        }

        void Mapper::add(pcl::PointCloud<PointType>::ConstPtr& pc, double time, bool downsample){
            if(pc->points.size() < 1) return;

            // If map doesn't exists, build one
            if(not this->exists()) this->build(pc);
            else this->add_pointcloud(pc, downsample);

            this->last_map_time = time;
        }

    // private

        void Mapper::build(pcl::PointCloud<PointType>::ConstPtr& pc){
            MapPoints map_vec;
            map_vec.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++)
                map_vec.emplace_back( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Build(map_vec);
        }

        void Mapper::add_pointcloud(pcl::PointCloud<PointType>::ConstPtr& pc, bool downsample){
            MapPoints map_vec;
            map_vec.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++)
                map_vec.emplace_back( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Add_Points(map_vec, downsample);
        }

        Match Mapper::match_plane(Eigen::Vector4f& p) {

            // Find k nearest points
            MapPoints near_points;
            std::vector<float> pointSearchSqDis(5/*Config.NUM_MATCH_POINTS*/);
            this->map->Nearest_Search(MapPoint(p(0), p(1), p(2)), 5/*Config.NUM_MATCH_POINTS*/, near_points, pointSearchSqDis);

            // Construct a plane fitting between them
            return Match(p.head(3), Plane (near_points, pointSearchSqDis));
        }
