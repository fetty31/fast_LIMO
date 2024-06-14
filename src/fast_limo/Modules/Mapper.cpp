#include "fast_limo/Modules/Mapper.hpp"

// class fast_limo::Mapper
    // public

        Mapper::Mapper() : last_map_time(-1.), num_threads_(1){
            this->map = KD_TREE<MapPoint>::Ptr (new KD_TREE<MapPoint>(0.3, 0.6, 0.2));
            /*To DO:
                - make KD_TREE parameters into shared config obj
            */
        }

        void Mapper::set_num_threads(int n){
            if(n < 1) return;
            this->num_threads_ = n;
        }

        void Mapper::set_match_points(int n){
            this->NUM_MATCH_POINTS_ = n;
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

        Matches Mapper::match(State s, pcl::PointCloud<PointType>::ConstPtr& pc){
            Matches matches;
            if(not this->exists()) return matches;
            matches.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++){
                
                Eigen::Vector4f bl4_point(pc->points[i].x, pc->points[i].y, pc->points[i].z, 1.); // base link 4d point
                Eigen::Vector4f global_point = s.get_RT() * bl4_point;                            // global 4d point == [x', y', z', 1.0]
                Match match = this->match_plane(global_point);                                    // point-to-plane match

                Eigen::Vector3f bl_point(pc->points[i].x, pc->points[i].y, pc->points[i].z);      // base link 3d point

                if(match.lisanAlGaib() 
                /* && bl_point.norm() < MIN_DIST2MATCH*/) matches.push_back(match); // if match is chosen, push it
            }

            return matches;
        }

        void Mapper::add(pcl::PointCloud<PointType>::Ptr& pc, double time, bool downsample){
            if(pc->points.size() < 1) return;

            // If map doesn't exists, build one
            if(not this->exists()) this->build(pc);
            else this->add_pointcloud(pc, downsample);

            this->last_map_time = time;
        }

    // private

        void Mapper::build(pcl::PointCloud<PointType>::Ptr& pc){
            MapPoints map_vec;
            map_vec.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size (); i++)
                map_vec.emplace_back( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Build(map_vec);
        }

        void Mapper::add_pointcloud(pcl::PointCloud<PointType>::Ptr& pc, bool downsample){
            MapPoints map_vec;
            map_vec.reserve(pc->points.size());

            #pragma omp parallel for num_threads(this->num_threads_)
            for(int i = 0; i < pc->points.size(); i++)
                map_vec.emplace_back( pc->points[i].x, pc->points[i].y, pc->points[i].z );

            this->map->Add_Points(map_vec, downsample);
        }

        Match Mapper::match_plane(Eigen::Vector4f& p) {

            // Find k nearest points
            MapPoints near_points;
            std::vector<float> pointSearchSqDis(this->NUM_MATCH_POINTS_);
            this->map->Nearest_Search(MapPoint(p(0), p(1), p(2)), this->NUM_MATCH_POINTS_, near_points, pointSearchSqDis);

            // Construct a plane fitting between them
            return Match(p.head(3), Plane (near_points, pointSearchSqDis));
        }
