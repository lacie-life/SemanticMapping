#include "QPCLVisual.h"

#include <QString>
#include <QDebug>

#include <vtkCamera.h>
#include <vtkDataSetMapper.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>

#include <pcl/io/pcd_io.h>

QPCLVisual::QPCLVisual(QWidget* parent, AppModel *model)
    : QVTKOpenGLNativeWidget(parent)
{
    m_model = model;
    m_count = 0;

    // set up point
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Set up the QVTK window
    renderer = vtkNew<vtkRenderer>();
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
//    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
                                                       "viewer",false));
    // PCL Viewer setting
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
}

void QPCLVisual::openPointCloud(QString path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(path.toStdString(),*_cloud);
    viewer->addPointCloud(_cloud);

    emit updateViewer();
}

void QPCLVisual::updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw)
{
    CONSOLE << "Update point";

    ++m_count;

    if(Tcw.log() == Sophus::SE3f().log())
    {
        cloud->clear();
        cloud->reserve(0);

        return;
    }

    auto Twc = Tcw.inverse();
    auto quat = Twc.unit_quaternion();
    auto trans = Twc.translation();

    // Add new point cloud
    cloud->resize(cloud->size() + frameCloud->size());
    std::copy(frameCloud->begin(), frameCloud->end(), this->cloud->end() - frameCloud->size());

    if (this->m_count % 20 == 0) {
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(0.02, 0.02, 0.02);
        filter.setInputCloud(cloud);
        filter.filter(*cloud);
        viewer->removeAllPointClouds();
        viewer->addPointCloud(this->cloud, "cloud");
    } else {
        viewer->addPointCloud(frameCloud, "cloud_" + std::to_string(this->m_count));
    }

    emit signalShowPtsFinished();
}

double QPCLVisual::curTime()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::time_point_cast<std::chrono::duration<double>>(now)
        .time_since_epoch()
                                                                      .count();
}

void QPCLVisual::renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw)
{

}


