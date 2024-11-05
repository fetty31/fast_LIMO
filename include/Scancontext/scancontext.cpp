#include "scancontext.hpp"

// class ScanContext
    // public:

        ScanContext::ScanContext() : sc_result(100000.0f) {
            this->NUM_EXCLUDE_RECENT        = 50;
            this->NUM_CANDIDATES_FROM_TREE  = 10;
            this->PC_NUM_RING               = 20;
            this->PC_NUM_SECTOR             = 60;
            this->PC_MAX_RADIUS             = 80.0f;
            this->SC_THRESHOLD              = 0.2f;
            this->SEARCH_RATIO              = 0.1f;
        }

        ScanContext::ScanContext(int num_exclude_recent, int num_candidates_tree, int pc_num_ring,
                    int pc_num_sector, float pc_max_radius, float sc_thres, float search_ratio)
                    : sc_result(100000.0f) 
        {
            this->NUM_EXCLUDE_RECENT        = num_exclude_recent;
            this->NUM_CANDIDATES_FROM_TREE  = num_candidates_tree;
            this->PC_NUM_RING               = pc_num_ring;
            this->PC_NUM_SECTOR             = pc_num_sector;
            this->PC_MAX_RADIUS             = pc_max_radius;
            this->SC_THRESHOLD              = sc_thres;
            this->SEARCH_RATIO              = search_ratio;
        }


        void ScanContext::makeAndSaveScanContextAndKeys(pcl::PointCloud<SCPointType>& scan){
            
            Eigen::MatrixXf sc = makeScanContext(scan); // scan context
            Eigen::VectorXf ringkey = makeRingKeyFromContext(sc);
            // Eigen::VectorXf sectorkey = makeSectorKeyFromContext(sc);

            std::vector<float> pc_invkeys(ringkey.data(), ringkey.data()+ringkey.size()); // polar context invariant keys

            this->buff_mtx.lock();

            contexts_.push_back(sc);
            invkeys_.push_back(ringkey);
            // vkeys_.push_back(sectorkey);
            invkeys_matrix_.push_back(pc_invkeys);

            this->buff_mtx.unlock();

        }

        std::pair<int, float> ScanContext::detectLoopClosureID(){

            int loop_id = -1;

            if(invkeys_matrix_.size() < NUM_EXCLUDE_RECENT || contexts_.size() < 1){
                std::pair<int, float> output {loop_id, 0.0};
                return output;
            }

            this->buff_mtx.lock();
            Eigen::VectorXf current_key = invkeys_.back(); // current observation key (query)
            Eigen::MatrixXf current_sc = contexts_.back(); // current obs. context (query)

            // STEP 1: candidates from ringkey tree
            if( this->time2resetTree() ){
                this->invkeys_to_search_.clear();
                this->invkeys_to_search_.assign( invkeys_matrix_.begin(), invkeys_matrix_.end() - NUM_EXCLUDE_RECENT);

                this->tree_.reset();
                this->tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING, invkeys_to_search_, 10/*leaf size*/);
            }

            this->buff_mtx.unlock();

            std::vector<size_t> candidate_idxs(NUM_CANDIDATES_FROM_TREE);
            std::vector<float> candidate_dists(NUM_CANDIDATES_FROM_TREE);

            nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
            knnsearch_result.init(&candidate_idxs[0], &candidate_dists[0]);
            this->tree_->index->findNeighbors(knnsearch_result, &current_key(0), nanoflann::SearchParams(10));

            // STEP 2: pairwise distance (find column-wise best-fit using cosine dist)
            float min_dist = 100000000.0f;
            int nn_align = 0;
            size_t nn_id = 0;

            this->buff_mtx.lock();
            for(size_t id=0; id < candidate_idxs.size(); id++){
                Eigen::MatrixXf candidate_sc = contexts_[candidate_idxs[id]];
                std::pair<float, int> sc_dist = distanceBtnScanContext(current_sc, candidate_sc);

                float candidate_dist = sc_dist.first;
                int candidate_align = sc_dist.second;

                if(candidate_dist < min_dist){
                    min_dist = candidate_dist;
                    nn_align = candidate_align;
                    nn_id = candidate_idxs[id];
                }
            }
            this->buff_mtx.unlock();

            // std::cout << "SCAN CONTEXT: minimum distance: " << min_dist << std::endl;
            // std::cout << "SCAN CONTEXT: best index: " << static_cast<int>(nn_id) << std::endl;

            if( min_dist < SC_THRESHOLD){
                loop_id = static_cast<int>(nn_id);

                // std::cout << "------------------------------------------------------------------------------\n";
                // std::cout << "--------------------- SCAN CONTEXT: LOOP FOUND! ------------------------------\n";
                // std::cout << "------------------------------------------------------------------------------\n";

            }
            // else{
            //     std::cout << "--------------------- SCAN CONTEXT: NO LOOP FOUND!----------------------------\n";
            // }

            this->sc_result   = min_dist;
            this->sc_best_idx = static_cast<int>(nn_id);

            float yaw_diff = nn_align * 2.0*M_PI/static_cast<float>(PC_NUM_SECTOR);
            std::pair<int, float> output {loop_id, yaw_diff};

            return output;
        }

        float ScanContext::getScanContextResult(){
            return this->sc_result;
        }

        int ScanContext::getScanContextIndex(){
            return this->sc_best_idx;
        }


    // private:

        Eigen::MatrixXf ScanContext::makeScanContext(pcl::PointCloud<SCPointType>& scan){
            
            int scan_size = scan.size();

            /*To DO:
                - change context value from pt.z to height difference (e.g. pt.z - min(scan.pt.z))
            */

            Eigen::MatrixXf sc = -1000.0f * Eigen::MatrixXf::Ones(PC_NUM_RING, PC_NUM_SECTOR);
            for(int id=0; id < scan_size; id++){
                SCPointType pt = scan.points[id];

                float azim_range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
                float azim_angle = this->getAzimuth(pt);

                if(azim_range > PC_MAX_RADIUS)
                    continue;

                int ring_id   = std::max( std::min(PC_NUM_RING,   int(ceil( (azim_range/PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
                int sector_id = std::max( std::min(PC_NUM_SECTOR, int(ceil( (azim_angle/(2.0*M_PI)) * PC_NUM_SECTOR )) ), 1 );

                if( sc(ring_id-1, sector_id-1) < pt.z )
                    sc(ring_id-1, sector_id-1) = pt.z;
            }

            for(int i=0; i<sc.rows(); i++)
                for(int j=0; j<sc.cols(); j++)
                    if(sc(i, j) <= -1000.0f) sc(i, j) = 0.0f;

            return sc;
        }

        std::pair<float, int> ScanContext::distanceBtnScanContext(Eigen::MatrixXf &_sc1, Eigen::MatrixXf &_sc2){

            // 1. fast align using variant key (not in original IROS18)
            Eigen::VectorXf vkey_sc1 = makeSectorKeyFromContext( _sc1 );
            Eigen::VectorXf vkey_sc2 = makeSectorKeyFromContext( _sc2 );
            
            int argmin_vkey_shift = this->fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

            const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
            std::vector<int> shift_idx_search_space { argmin_vkey_shift };
            for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
            {
                shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
                shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
            }
            std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

            // 2. fast columnwise diff 
            int argmin_shift = 0;
            float min_sc_dist = 10000000;
            for ( int num_shift: shift_idx_search_space )
            {
                Eigen::MatrixXf sc2_shifted = this->circShift(_sc2, num_shift);
                float cur_sc_dist = this->distanceDirectScanContext( _sc1, sc2_shifted );
                if( cur_sc_dist < min_sc_dist )
                {
                    argmin_shift = num_shift;
                    min_sc_dist = cur_sc_dist;
                }
            }

            return std::make_pair(min_sc_dist, argmin_shift);

        }

        Eigen::VectorXf ScanContext::makeRingKeyFromContext(Eigen::MatrixXf& context){
            Eigen::VectorXf inv_key(context.rows());
            for(int i=0; i < context.rows(); i++)
                inv_key(i) = context.row(i).mean();

            return inv_key;
        }

        Eigen::VectorXf ScanContext::makeSectorKeyFromContext(Eigen::MatrixXf& context){
            Eigen::VectorXf v_key(context.cols());
            for(int i=0; i < context.cols(); i++)
                v_key(i) = context.col(i).mean();
            
            return v_key;
        }

        float ScanContext::getAzimuth(SCPointType& pt){
            float azimuth = std::atan2(pt.y, pt.x);
            if(azimuth < 0)
                return azimuth + 2.0f*M_PI;
            else
                return azimuth;
        }

        bool ScanContext::time2resetTree(){
            return true;
        }


        Eigen::MatrixXf ScanContext::circShift(Eigen::MatrixXf _mat, int _num_shift){

            // shift columns to right direction 
            assert(_num_shift >= 0);

            if( _num_shift == 0 )
            {
                Eigen::MatrixXf shifted_mat( _mat );
                return shifted_mat; // Early return 
            }

            Eigen::MatrixXf shifted_mat = Eigen::MatrixXf::Zero( _mat.rows(), _mat.cols() );
            for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
            {
                int new_location = (col_idx + _num_shift) % _mat.cols();
                shifted_mat.col(new_location) = _mat.col(col_idx);
            }

            return shifted_mat;

        }

        int ScanContext::fastAlignUsingVkey(Eigen::VectorXf &_vkey1, Eigen::VectorXf &_vkey2){
            int argmin_vkey_shift = 0;
            float min_veky_diff_norm = 10000000.0f;
            for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
            {
                Eigen::MatrixXf vkey2_shifted = this->circShift(_vkey2, shift_idx);

                Eigen::MatrixXf vkey_diff = _vkey1 - vkey2_shifted;

                float cur_diff_norm = vkey_diff.norm();
                if( cur_diff_norm < min_veky_diff_norm )
                {
                    argmin_vkey_shift = shift_idx;
                    min_veky_diff_norm = cur_diff_norm;
                }
            }

            return argmin_vkey_shift;
        }

        float ScanContext::distanceDirectScanContext(Eigen::MatrixXf &_sc1, Eigen::MatrixXf &_sc2){
            int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
            float sum_sector_similarity = 0.0f;
            for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
            {
                Eigen::VectorXf col_sc1 = _sc1.col(col_idx);
                Eigen::VectorXf col_sc2 = _sc2.col(col_idx);
                
                if( col_sc1.norm() == 0 | col_sc2.norm() == 0 )
                    continue; // don't count this sector pair. 

                float sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

                sum_sector_similarity = sum_sector_similarity + sector_similarity;
                num_eff_cols = num_eff_cols + 1;
            }
            
            float sc_sim = sum_sector_similarity / num_eff_cols;
            return 1.0 - sc_sim;

        }