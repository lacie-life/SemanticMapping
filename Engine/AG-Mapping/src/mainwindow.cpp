#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    PCLWidget = new QPCLVisual(nullptr);

    ui->pcl_widget->setRenderWindow(PCLWidget->viewer->getRenderWindow());
    PCLWidget->viewer->setupInteractor(ui->pcl_widget->interactor(), ui->pcl_widget->renderWindow());
    ui->pcl_widget->update();

    connect(ui->openPCD, &QPushButton::clicked, this, &MainWindow::chooseFile);
    connect(this, &MainWindow::pcdFile, PCLWidget, &QPCLVisual::openPointaCloud);
    connect(PCLWidget, &QPCLVisual::updateViewer, this, &MainWindow::updatePCLWidget);
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
