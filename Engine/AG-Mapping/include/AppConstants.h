#ifndef APPCONSTANTS_H
#define APPCONSTANTS_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QColor>
#include <QMutex>
#include <QCoreApplication>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "

#endif

#ifndef BUILD_DIR

#define BUILD_DIR QCoreApplication::applicationDirPath()

#endif

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#endif // APPCONSTANTS_H
