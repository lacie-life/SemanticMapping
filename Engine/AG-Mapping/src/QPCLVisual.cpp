#include "QPCLVisual.h"

#include <QString>

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
    // Set up the QVTK window
    renderer = vtkNew<vtkRenderer>();
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
//    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
                                                       "viewer",false));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
}

void QPCLVisual::openPointaCloud(QString path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(path.toStdString(),*_cloud);
    viewer->addPointCloud(_cloud);

    emit updateViewer();
}


