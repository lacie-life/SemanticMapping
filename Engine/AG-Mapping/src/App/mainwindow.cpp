#include "mainwindow.h"
#include "AppConstants.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QThread>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_model = AppModel::instance();

    m_configDiaglog = new QConfigDialog(nullptr, m_model);
    m_slamWidget = new QSLAMWidget(nullptr);
    m_intro = new QIntroduction(nullptr);

    m_stackWidget = new QStackedWidget();

    m_stackWidget->addWidget(m_slamWidget);
    m_stackWidget->addWidget(m_intro);

    setCentralWidget(m_stackWidget);

    m_stackWidget->setCurrentWidget(m_intro);

    connect(m_intro, &QIntroduction::systemConfigOpen, this->m_configDiaglog, &QConfigDialog::open);
    connect(m_configDiaglog, &QConfigDialog::configDone, this, &MainWindow::SLAMInforDisplay);

    connect(m_intro, &QIntroduction::actionRunClicked, this, [this]()
    {
        CONSOLE << "Are u here?";

        m_stackWidget->setCurrentWidget(m_slamWidget);

        QThread::msleep(100);

        m_model->SLAM_Run();

        connect(this->m_model, &AppModel::updateTrajactory, this->m_slamWidget, &QSLAMWidget::updateTraj);
    });

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::SLAMInforDisplay()
{
}
