//
// Created by lacie on 15/01/2023.
//

#include "PointCloudMapping.h"

#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/hal/interface.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>

#include "Converter.h"

#include <boost/make_shared.hpp>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);
    globalMap = boost::make_shared<PointCloud>();

    viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout << "receive a keyframe, id = " << kf->mnId << endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}

pcl::PointCloud<PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp(new PointCloud());

    if(kf->GetPose().log() == Sophus::SE3f().log())
    {
        globalMap->clear();
        globalMap->reserve(0);
        return tmp;
    }

    // TODO: Convert point cloud
    // point cloud is null ptr
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;

            PointT _p;
            _p.z = d;
            _p.x = (n - kf->cx) * _p.z * kf->invfx;
            _p.y = (m - kf->cy) * _p.z * kf->invfy;

            _p.b = color.ptr<uchar>(m)[n*3];
            _p.g = color.ptr<uchar>(m)[n*3+1];
            _p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(_p);
        }
    }

    std::cout << tmp->size() << std::endl;

    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());

    PointCloud::Ptr cloud(new PointCloud);
    cloud->resize(tmp->size());

    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix(), true);

    cloud->is_dense = false;

    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;

    return cloud;

}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");

    while(1)
    {
        ++this->m_count;
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }

        for ( size_t i = lastKeyframeSize; i < N ; i++ )
        {
            PointCloud::Ptr p(new PointCloud);
            p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            *globalMap += *p;
        }

        PointCloud::Ptr tmp(new PointCloud());
        tmp->resize(globalMap->size());
        voxel.setInputCloud(globalMap);
        voxel.filter(*tmp);
        globalMap->swap(*tmp);
        viewer.showCloud(globalMap);

        cout << "show global map, size=" << globalMap->points.size() << endl;

        lastKeyframeSize = N;
    }

    pcl::io::savePCDFile("result.pcd", *globalMap);
    std::cout << "Final point cloud saved!!!" << std::endl;
}

