#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include "Converter.h"
#include "ObjectDatabase.h"
#include "MergeSG.h"
#include "YoloDetection.h"

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <condition_variable>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace AG_MAPPING;

class PointCloudMapping
{

public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    /**
     * @brief Construct a new Point Cloud Mapping object
     * 
     * @param resolution_ 
     */
    PointCloudMapping(double resolution_, std::string modelPath);

    /**
     * @brief Insert new keyframe and RGB-D image, and trigger pointcloud generation process
     * 
     * @param kf keyframe 
     * @param color rgb image 
     * @param depth depth image
     */
    void insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);

    /**
     * @brief Shuts down all threads properly
     */
    void shutdown();

    MergeSG* mpMergeSG;
    YoloDetection* mDetector;

protected:
    PointCloud::Ptr globalMap;

    boost::shared_ptr<thread> viewerThread;
    boost::shared_ptr<thread> pclThread;

    bool shutDownFlag = false;
    std::mutex shutDownMutex;
    std::mutex keyFrameUpdateMutex;
    std::condition_variable keyFrameUpdated;
    std::condition_variable newMaskArrived;

    // Data to generate point clouds
    std::map<int, cv::Mat> maskMap;
    std::vector<KeyFrame *> keyframes;
    std::vector<cv::Mat> colorImgs;
    std::vector<cv::Mat> depthImgs;
    std::vector<int> imgMasksSeq;
    std::mutex keyframeMutex;
    std::mutex maskMutex;
    uint16_t lastKeyframeSize = 0;

    double resolution = 0.005;
    pcl::VoxelGrid<PointT> voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor;

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth);
    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat color, cv::Mat depth, std::vector<Object>& objects);
    PointCloud::Ptr generatePointCloudWithDynamicObject(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    /**
     * @brief Runs all functionalities in a new thread
     */
    void publisher();

    /**
     * @brief Generates and publishes the pointcloud in a separate thread
     * 
     * @param N Total number of all (keyframes, RGB-D images) inserted
     */
    void generateAndPublishPointCloud(size_t N);

    void generateAndPublishOctoMap(size_t N);

    /**
     * @brief Calculates transformation matrix based on camera pose
     * and publishes it as a ROS tf message to transfrom the pointcloud to world coordinates
     * 
     * @param cameraPose camera pose generated by ORB-SLAM3
     */
    void broadcastTransformMat(Eigen::Isometry3d cameraPose);

    bool checkDynamicPoint(cv::Point2f pt, std::vector<Object> objects);
};

#endif // POINTCLOUDMAPPING_H