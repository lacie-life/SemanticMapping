#ifndef APPMODEL_H
#define APPMODEL_H

#include <QObject>
#include <QMutex>

#include "AppEnums.h"

class AppModel : public QObject
{
    Q_OBJECT
public:
    static AppModel *instance();
    void set_slam_type(AppEnums::VSLAM_TYPE type);
    AppEnums::VSLAM_TYPE get_slam_type();

public slots:
    void SLAM_Run(QStringList settingPaths);

private:
    AppModel(QObject* parent = nullptr);
    AppModel(const AppModel& _other) = delete;
    void operator =(const AppModel& _other) = delete;

public:
    QString m_seqPath;

    QString m_ImgPath;
    QString m_LeftImgPath;
    QString m_RightImgPath;

    QString m_AssoPath;
    QString m_LeftAssoPath;
    QString m_RightAssoPath;

    QString m_IMUPath;

    QString m_vocPath;
    QString m_settingPath;

    QString m_GTPath;

    QString m_classes;
    QString m_modelConfig;
    QString m_modelWeights;

private:
    static AppModel* m_instance;
    static QMutex m_lock;

    static AppEnums::APP_STATE m_state;
    static AppEnums::VSLAM_TYPE m_slam_type;
    static AppEnums::SLAM_STATE m_slam_state;
};

#endif // APPMODEL_H
