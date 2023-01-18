//
// Created by lacie on 15/01/2023.
//

#ifndef AG_MAPPING_POINTCLOUDMAPPING_H
#define AG_MAPPING_POINTCLOUDMAPPING_H

#include "System.h"
#include "KeyFrame.h"

#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/cloud_viewer.h"

#include <condition_variable>

#include <boost/make_shared.hpp>
#include <boost//format.hpp>
#include <Eigen/Geometry>

using namespace  ORB_SLAM3;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointCloudMapping {

public:

    PointCloudMapping(double resolution_);

    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    void shutdown();
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;

    int mCurrentMapId;
    int mLastMpId;

    std::size_t m_count;

    bool shutDownFlag = false;
    mutex shutDownMutex;

    condition_variable keyFrameUpdated;
    mutex keyFrameUpdateMutex;

    // data to generate point clouds
    std::vector<KeyFrame*> keyframes;
    std::vector<cv::Mat> colorImgs;
    std::vector<cv::Mat> depthImgs;

    std::mutex keyframeMutex;
    uint16_t lastKeyframeSize = 0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};

#endif //AG_MAPPING_POINTCLOUDMAPPING_H
