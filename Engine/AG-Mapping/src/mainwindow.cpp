#include "mainwindow.h"
#include "AppConstants.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_model = AppModel::instance();

    this->ui->slam_type_label->hide();
    this->ui->run_action->hide();

    m_PCLWidget = new QPCLVisual(nullptr, m_model);
    m_configDiaglog = new QConfigDialog(nullptr, m_model);

    ui->pcl_widget->setRenderWindow(m_PCLWidget->viewer->getRenderWindow());
    m_PCLWidget->viewer->setupInteractor(ui->pcl_widget->interactor(), ui->pcl_widget->renderWindow());
    ui->pcl_widget->update();

    connect(ui->openPCD, &QPushButton::clicked, this, &MainWindow::chooseFile);
    connect(this, &MainWindow::pcdFile, m_PCLWidget, &QPCLVisual::openPointCloud);
    connect(m_PCLWidget, &QPCLVisual::updateViewer, this, &MainWindow::updatePCLWidget);
    connect(ui->system_config, &QPushButton::clicked, this->m_configDiaglog, &QConfigDialog::open);
    connect(m_configDiaglog, &QConfigDialog::configDone, this, &MainWindow::SLAMInforDisplay);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::chooseFile()
{
    QString file_name = QFileDialog::getOpenFileName(this, "Open a file", "${HOME}");
    CONSOLE << file_name;
    emit pcdFile(file_name);
}

void MainWindow::updatePCLWidget()
{
    CONSOLE << "Update PCL Viewer";
    ui->pcl_widget->update();
}

void MainWindow::SLAMInforDisplay()
{
    this->ui->slam_type_label->show();
    this->ui->run_action->show();

    switch (m_model->get_slam_type()) {
    case static_cast<int>(AppEnums::VSLAM_TYPE::MONO):
        this->ui->slam_type_label->setText("SLAM Type Mono-VO");
        break;
    case static_cast<int>(AppEnums::VSLAM_TYPE::MONO_IMU):
        this->ui->slam_type_label->setText("SLAM Type Mono-VIO");
        break;
    case static_cast<int>(AppEnums::VSLAM_TYPE::STEREO):
        this->ui->slam_type_label->setText("SLAM Type Stereo-VO");
        break;
    case static_cast<int>(AppEnums::VSLAM_TYPE::STEREO_IMU):
        this->ui->slam_type_label->setText("SLAM Type Stereo-VIO");
        break;
    default:
        break;
    }
}
