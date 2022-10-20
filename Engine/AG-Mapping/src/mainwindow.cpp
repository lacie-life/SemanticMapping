#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkNew<vtkRenderer> renderer;
    renderWindow->AddRenderer(renderer);

    std::string name = "/home/lacie/Github/Dataset/Scan_2012_1_10_004_Structured.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(name,*cloud);

    //viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow,
                                                       "viewer",false));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addPointCloud(cloud);
    //ui->Viewer_widget->renderWindow()->AddRenderer(renderer);
    ui->Viewer_widget->setRenderWindow(viewer->getRenderWindow());
    ui->Viewer_widget->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
