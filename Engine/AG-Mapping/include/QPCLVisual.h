#ifndef QPCLVISUAL_H
#define QPCLVISUAL_H

#include <QString>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkDataSet.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

class QPCLVisual : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    QPCLVisual(QWidget* parent = nullptr);

    void openPointaCloud(QString path);

signals:
    void updateViewer();

public:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    vtkNew<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;

};

#endif // QPCLVISUAL_H
