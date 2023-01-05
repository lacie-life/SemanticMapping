#ifndef APPCONSTANTS_H
#define APPCONSTANTS_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QColor>
#include <QMutex>
#include <QCoreApplication>
#include <QDebug>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "

#endif

#ifndef BUILD_DIR

#define BUILD_DIR QCoreApplication::applicationDirPath()

#endif

// GUI
#define BACKGOUND ":/images/data/res/background.jpg"
#define APP_ICON ":/images/data/res/icon.png"

#define PARAM_PATH "/home/lacie/Github/AG-Mapping/Engine/AG-Mapping/config/stereo.yaml"
#define IMAGE_PATH "/home/lacie/Github/Data/EuRoC/MH_01_easy/mav0"
#define TIMESTAMP_PATH "/home/lacie/Github/AG-Mapping/data/euroc_data_timestamp/MH01.txt"

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#endif // APPCONSTANTS_H
