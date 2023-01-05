#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include "AppConstants.h"
#include "QSLAM.h"

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    QSLAM *m_slam = new QSLAM();

    m_slam->init(PARAM_PATH);

    QStringList temp;
    temp.append(QString(IMAGE_PATH) + "/imu0/data.csv");
    temp.append(QString(IMAGE_PATH) + "/cam0/data");
    temp.append(QString(IMAGE_PATH) + "/cam1/data");
    temp.append(QString(IMAGE_PATH) + "/timestamp/data/time.txt");

    m_slam->run(temp);

    return a.exec();
}
