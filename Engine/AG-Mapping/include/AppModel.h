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

signals:

private:
    AppModel(QObject* parent = nullptr);
    AppModel(const AppModel& _other) = delete;
    void operator =(const AppModel& _other) = delete;

public:
    void set_slam_type(AppEnums::VSLAM_TYPE type);
    AppEnums::VSLAM_TYPE get_slam_type();

private:
    static AppModel* m_instance;
    static QMutex m_lock;

    static AppEnums::APP_STATE m_state;
    static AppEnums::VSLAM_TYPE m_slam_type;
    static AppEnums::SLAM_STATE m_slam_state;
};

#endif // APPMODEL_H
