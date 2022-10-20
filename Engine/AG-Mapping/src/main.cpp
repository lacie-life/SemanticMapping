#include "mainwindow.h"
#include "QVTKOpenglWindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);
    QVTKOpenGLWindow w;
    w.show();
    return a.exec();
}
