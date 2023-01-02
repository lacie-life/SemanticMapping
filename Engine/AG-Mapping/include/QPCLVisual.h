#ifndef QPCLVISUAL_H
#define QPCLVISUAL_H

#include "AppModel.h"
#include "AppConstants.h"

#include <QString>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkDataSet.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>

#include <boost/shared_ptr.hpp>

#include "sophus/se3.hpp"

class QPCLVisual : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    QPCLVisual(QWidget* parent = nullptr, AppModel *model = nullptr);

    void openPointCloud(QString path);

    void updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

    double curTime();

public slots:
    void renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

signals:
    void updateViewer();
    void signalShowPtsFinished();

public:
    AppModel *m_model;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    vtkNew<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;

    std::size_t m_count;

    AppEnums::PCL_VISUAL_MODE m_vis_mode = AppEnums::PCL_VISUAL_MODE::RENDER_MODE;

};

#endif // QPCLVISUAL_H
