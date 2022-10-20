#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/obj_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/parse.h"

#include "QVTKOpenGLNativeWidget.h"
#include "vtkCamera.h"
#include "vtkCubeSource.h"
#include "vtkDataObjectToTable.h"
#include "vtkElevationFilter.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkNamedColors.h"
#include "vtkNew.h"
#include "vtkPolyDataMapper.h"
#include "vtkQtTableView.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkVersion.h"
#include "pcl/visualization/pcl_visualizer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};
#endif // MAINWINDOW_H
