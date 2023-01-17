//
// Created by lacie on 15/01/2023.
//

#include "Converter.h"
#include "PointCloudMapping.h"

#include <KeyFrame.h>
#include <boost/make_shared.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
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
    keyframes.push_back(kf );
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp(new PointCloud());

    tmp->reserve(depth.rows * depth.cols * 0.9);

    auto Twc = kf->GetPose();

    Eigen::Matrix<double,3,3> R;
    R << Twc.at<float>(0,0), Twc.at<float>(0,1), Twc.at<float>(0,2),
            Twc.at<float>(1,0), Twc.at<float>(1,1), Twc.at<float>(1,2),
            Twc.at<float>(2,0), Twc.at<float>(2,1), Twc.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(Twc.at<float>(0,3), Twc.at<float>(1,3), Twc.at<float>(2,3));

    // point cloud is null ptr
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;

//            float scalar = float(d);
//            Eigen::Matrix<double, 3, 1> p((n - kf->cx) * scalar / kf->fx, (m - kf->cy) * scalar / kf->fy, scalar);
//            Eigen::Matrix<double, 3, 1> p_w = R * p + t;
//
//            PointT _p;
//            _p.z = p_w(2);
//            _p.x = p_w(0);
//            _p.y = p_w(1);

            PointT _p;
            _p.z = d;
            _p.x = ( n - kf->cx) * _p.z / kf->fx;
            _p.y = ( m - kf->cy) * _p.z / kf->fy;

            _p.b = color.ptr<uchar>(m)[n*3];
            _p.g = color.ptr<uchar>(m)[n*3+1];
            _p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(_p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());

    std::cout << T.inverse().matrix() << std::endl;

    PointCloud::Ptr cloud(new PointCloud);
    cloud->resize(tmp->size());

//    std::copy(tmp->begin(), tmp->end(), cloud->begin());

    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix(), true);
    cloud->is_dense = false;

    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
    return cloud;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("Viewer");

//    this->m_viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
//
//    m_viewer->removeAllPointClouds();
//
//    this->m_viewer->setSize(1000, 640);
//    this->m_viewer->setBackgroundColor(0.9f, 0.9f, 0.9f);
//    this->m_count = 0;
//
//    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
//    origin.pretranslate(Eigen::Vector3f::Zero());
//    this->m_viewer->addCoordinateSystem(0.5, Eigen::Affine3f(origin.affine()), "origin");
//
//    m_viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
//    m_viewer->spinOnce();

    while(1)
    {
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

        for (size_t i = lastKeyframeSize; i<N ; i++)
        {
            PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            this->globalMap->resize(this->globalMap->size() + p->size());
            std::copy(p->begin(), p->end(), this->globalMap->end() - p->size());
        }

        PointCloud::Ptr tmp(new PointCloud());

        voxel.setInputCloud(globalMap);
        voxel.filter(*tmp);
        globalMap->swap(*tmp);
        viewer.showCloud(globalMap);

//        this->m_viewer->removeAllPointClouds();
//        this->m_viewer->addPointCloud(globalMap);
//        this->globalMap->clear();
//        this->m_viewer->spinOnce();

        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
    pcl::io::savePCDFileASCII("map.pcd", *globalMap);
}
