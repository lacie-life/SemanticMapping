#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_PCLWidget = new QPCLVisual(nullptr);
    m_configDiaglog = new QConfigDialog(nullptr);

    ui->pcl_widget->setRenderWindow(m_PCLWidget->viewer->getRenderWindow());
    m_PCLWidget->viewer->setupInteractor(ui->pcl_widget->interactor(), ui->pcl_widget->renderWindow());
    ui->pcl_widget->update();

    connect(ui->openPCD, &QPushButton::clicked, this, &MainWindow::chooseFile);
    connect(this, &MainWindow::pcdFile, m_PCLWidget, &QPCLVisual::openPointaCloud);
    connect(m_PCLWidget, &QPCLVisual::updateViewer, this, &MainWindow::updatePCLWidget);
    connect(ui->system_config, &QPushButton::clicked, this->m_configDiaglog, &QConfigDialog::open);
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
