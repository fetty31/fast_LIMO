#ifndef __SCANCONTEXT_PLUSPLUS__
#define __SCANCONTEXT_PLUSPLUS__

#include <cmath>
#include <vector>
#include <mutex>
#include <memory>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

using KeyMat     = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;
using SCPointType  = pcl::PointXYZ;

class ScanContext {

    // VARIABLES

    public:
        int NUM_EXCLUDE_RECENT = 50;

    private:

        std::mutex buff_mtx;
        std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf>> contexts_;
        std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> invkeys_;    
        // std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> vkeys_;    

        KeyMat invkeys_matrix_;
        KeyMat invkeys_to_search_;

        std::unique_ptr<InvKeyTree> tree_;

    // FUNCTIONS

    public:
        ScanContext();

        void makeAndSaveScanContextAndKeys(pcl::PointCloud<SCPointType>& scan);
        std::pair<int, float> detectLoopClosureID();

    private:
        Eigen::MatrixXf makeScanContext(pcl::PointCloud<SCPointType>& scan);
        Eigen::VectorXf makeRingKeyFromContext(Eigen::MatrixXf& context);
        Eigen::VectorXf makeSectorKeyFromContext(Eigen::MatrixXf& context);

        float getAzimuth(SCPointType& pt);
        float distanceDirectScanContext(Eigen::MatrixXf &_sc1, Eigen::MatrixXf &_sc2);

        bool time2resetTree();

        int fastAlignUsingVkey(Eigen::VectorXf &_vkey1, Eigen::VectorXf &_vkey2);

        Eigen::MatrixXf circShift(Eigen::MatrixXf _mat, int _num_shift);

        std::pair<float, int> distanceBtnScanContext(Eigen::MatrixXf &_sc1, Eigen::MatrixXf &_sc2);
        
};

#endif