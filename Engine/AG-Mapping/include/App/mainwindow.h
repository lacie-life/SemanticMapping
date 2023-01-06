#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "QConfigDialog.h"
#include "AppModel.h"
#include "QSLAMWidget.h"
#include "QIntroduction.h"

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

public slots:
    void SLAMInforDisplay();

private:
    Ui::MainWindow *ui;
    QConfigDialog *m_configDiaglog;
    QSLAMWidget *m_slamWidget;
    QIntroduction *m_intro;

    AppModel *m_model;

};
#endif // MAINWINDOW_H
