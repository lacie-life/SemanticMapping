//
// Created by lacie on 15/01/2023.
//

#ifndef AG_MAPPING_POINTCLOUDMAPPING_H
#define AG_MAPPING_POINTCLOUDMAPPING_H

#include "System.h"

#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

#include <condition_variable>

#include <boost/make_shared.hpp>

using namespace  ORB_SLAM3;

class PointCloudMapping {

public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_);

    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    void shutdown();
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    std::shared_ptr<thread>  viewerThread;

    bool    shutDownFlag    =false;
    mutex   shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize = 0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};

#endif //AG_MAPPING_POINTCLOUDMAPPING_H
