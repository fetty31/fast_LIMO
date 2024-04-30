#include "fast_limo/Modules/Mapper.hpp"

Mapper::Mapper(int num_threads) : last_map_time(-1.), num_threads_(num_threads){
    this->map = KD_TREE<PointType>::Ptr (new KD_TREE<PointType>(0., 0., 0., 0., 0., 0., 0.));
}

// Public
        
bool Mapper::exists(){
    return this->map->size() > 1;
}

int Mapper::size(){
    return this->map->size();
}

double Mapper::last_time(){
    return this->last_map_time;
}

Matches Mapper::match(const State& s, pcl::PointCloud<PointType>::ConstPtr& pc){
    Matches matches;
    if(not this->exists()) return matches;
    matches.reserve(pc->points.size());

    #pragma omp parallel for num_threads(this->num_threads_)
    for(MapPoints::iterator map_it = pc->begin(); map_it != pc->end(); it++ ){
        Eigen::Vector3f point = s.get_RT() * map_it.getVector3fMap();
        Match match = this->match_plane()
    }
}

void Mapper::add(pcl::PointCloud<PointType>::ConstPtr& pc, double time, bool downsample=false){
    if(pc->points.size() < 1) return;

    // If map doesn't exists, build one
    if(not this->exists()) this->build(pc);
    else this->add_pointcloud(pc, downsample);

    this->last_map_time = time;
}

// Private

void Mapper::build(pcl::PointCloud<PointType>::ConstPtr& pc){
    MapPoints map_vec;
    map_vec.reserve(pc->points.size());
    for(MapPoints::iterator map_it = pc->begin(); map_it != pc->end(); it++ ) map_vec.push_back( 
                                                                                            map_it.getVector3fMap()[0], 
                                                                                            map_it.getVector3fMap()[1], 
                                                                                            map_it.getVector3fMap()[2]
                                                                                            );
    this->map->Build(map_vec);
}

void Mapper::add_pointcloud(pcl::PointCloud<PointType>::ConstPtr& pc, bool downsample=false){
    MapPoints map_vec;
    map_vec.reserve(pc->points.size());
    for(MapPoints::iterator map_it = pc->begin(); map_it != pc->end(); it++ )map_vec.push_back( 
                                                                                            map_it.getVector3fMap()[0], 
                                                                                            map_it.getVector3fMap()[1], 
                                                                                            map_it.getVector3fMap()[2]
                                                                                            );
    this->map->Add_Points(map_vec, downsample);
}

Match Mapper::match_plane(Eigen::Vector3f& p) {

    // Find k nearest points
    MapPoints near_points;
    std::vector<float> pointSearchSqDis(5/*Config.NUM_MATCH_POINTS*/);
    this->map->Nearest_Search(pcl::PointXYZ(p(0), p(1), p(2)), 5/*Config.NUM_MATCH_POINTS*/, near_points, pointSearchSqDis);

    // Construct a plane fitting between them
    return Match(p, Plane (near_points, pointSearchSqDis));
}
