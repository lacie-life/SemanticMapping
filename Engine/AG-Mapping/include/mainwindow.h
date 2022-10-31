#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "AppConstants.h"
#include "QPCLVisual.h"
#include "QConfigDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void pcdFile(QString path);

public slots:
    void chooseFile();
    void updatePCLWidget();

private:
    Ui::MainWindow *ui;
    QPCLVisual *m_PCLWidget;
    QConfigDialog *m_configDiaglog;
};
#endif // MAINWINDOW_H
