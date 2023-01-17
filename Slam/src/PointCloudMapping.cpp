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
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp(new PointCloud());
    tmp->reserve(depth.rows * depth.cols * 0.9);

    if(kf->GetPose().log() == Sophus::SE3f().log())
    {
        globalMap->clear();
        globalMap->reserve(0);
        // camera
        this->m_viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        this->m_viewer->spinOnce();
        return tmp;
    }

//    auto Twc = kf->GetPose().inverse();
//    auto quat = Twc.unit_quaternion().matrix();
//    auto trans = Twc.translation();
//
//    double test[4][4] = {{quat(0,0), quat(0,1), quat(0,2), trans(0,0)},
//                         {quat(1,0), quat(1,1), quat(1,2), trans(1,0)},
//                         {quat(2,0), quat(2,1), quat(2,2), trans(2,0)},
//                         {0, 0, 0, 1}};
//
//    cv::Mat _Twc = cv::Mat(cv::Size(4, 4), CV_32F, test);

    // TODO: Convert point cloud
    // point cloud is null ptr
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;

            // --- for cloud point ---
//            float scalar = float(d) / 1000.0;
//            Eigen::Vector3f p((n - kf->cx) * scalar / kf->fx, (m - kf->cy) * scalar / kf->fy, scalar);
//            Eigen::Vector3f p_w = quat * p + trans;
//
//            PointT _p;
//            _p.z = p_w(0);
//            _p.x = p_w(1);
//            _p.y = p_w(2);

            PointT _p;
            _p.z = d;
            _p.x = (n - kf->cx) * _p.z / kf->fx;
            _p.y = (m - kf->cy) * _p.z / kf->fy;

            _p.b = color.ptr<uchar>(m)[n*3];
            _p.g = color.ptr<uchar>(m)[n*3+1];
            _p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(_p);
        }
    }

    std::cout << tmp->size() << std::endl;

    g2o::SE3Quat testSE3 = ORB_SLAM3::Converter::toSE3Quat(kf->GetImuPose()); // Sophus::SE3f to g2o::SE3Quat
    cv::Mat testMat = ORB_SLAM3::Converter::toCvMat(testSE3); // g2o::SE3Quat to cv::Mat (like ORB_SLAM2)

    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(testMat);

    std::cout << T.matrix() << std::endl;

    PointCloud::Ptr cloud(new PointCloud);
    cloud->resize(tmp->size());

//    std::copy(tmp->begin(), tmp->end(), cloud->begin());

    pcl::transformPointCloud( *tmp, *cloud, T.matrix(), true);

    cloud->is_dense = false;

    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;

    return cloud;

}

void PointCloudMapping::viewer()
{
//    pcl::visualization::CloudViewer viewer("viewer");
    this->m_viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();

    m_viewer->removeAllPointClouds();

    this->m_viewer->setSize(1000, 640);
    this->m_viewer->setBackgroundColor(0.9f, 0.9f, 0.9f);
    this->m_count = 0;

    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
    origin.pretranslate(Eigen::Vector3f::Zero());
    this->m_viewer->addCoordinateSystem(0.5, Eigen::Affine3f(origin.affine()), "origin");

    m_viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    m_viewer->spinOnce();

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
            PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            this->globalMap->resize(this->globalMap->size() + p->size());
            std::copy(p->begin(), p->end(), this->globalMap->end() - p->size());
//            *globalMap += *p;
        }

        this->m_viewer->removeAllPointClouds();
        this->m_viewer->addPointCloud(globalMap);
//        this->globalMap->clear();
        this->m_viewer->spinOnce();

//        PointCloud::Ptr tmp(new PointCloud());
//        voxel.setInputCloud(globalMap);
//        voxel.filter(*tmp);
//        globalMap->swap(*tmp);
//        viewer.showCloud(globalMap);

        cout << "show global map, size=" << globalMap->points.size() << endl;

        lastKeyframeSize = N;
    }

    pcl::io::savePCDFile("result.pcd", *globalMap);
    std::cout << "Final point cloud saved!!!" << std::endl;
}

