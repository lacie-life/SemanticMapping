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

    m_configDiaglog = new QConfigDialog(nullptr, m_model);
    m_slamWidget = new QSLAMWidget(nullptr);
    m_intro = new QIntroduction(nullptr);

    ui->stackedWidget->addWidget(m_slamWidget);
    ui->stackedWidget->addWidget(m_intro);

    ui->stackedWidget->setCurrentWidget(m_intro);

    connect(ui->system_config, &QPushButton::clicked, this->m_configDiaglog, &QConfigDialog::open);
    connect(m_configDiaglog, &QConfigDialog::configDone, this, &MainWindow::SLAMInforDisplay);
    connect(ui->run_action, &QPushButton::clicked, this, [this]()
    {
        this->ui->stackedWidget->setCurrentWidget(m_slamWidget);
    });
//    connect(ui->run_action, &QPushButton::clicked, this->m_model, &AppModel::SLAM_Run);
}

MainWindow::~MainWindow()
{
    delete ui;
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
