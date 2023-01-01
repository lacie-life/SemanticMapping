#ifndef APPENUMS_H
#define APPENUMS_H

#include <QObject>

class AppEnums : public QObject
{
    Q_OBJECT
    Q_ENUMS(APP_STATE)
    Q_ENUMS(VSLAM_MODE)
    Q_ENUMS(VSLAM_STATE)
    Q_ENUMS(PCL_VISUAL_MODE)

public:

    enum APP_STATE
    {
        NONE_STATE = 0,
        PCL_VISUAL_STATE,
        VSLAM_STATE,
        END_STATE,
    };

    enum VSLAM_TYPE
    {
        NONE_TYPE = 0,
        MONO,
        MONO_IMU,
        STEREO,
        STEREO_IMU,
        MULTI_CAM,
        MULTI_CAM_IMU,
        END_TYPE,
    };

    enum SLAM_STATE
    {
        VSLAM_NONE_STATE = 0,
        RUNNING_STATE,
        PAUSE_STATE,
        STOP_STATE,
        VSLAM_END_STATE,
    };

    enum PCL_VISUAL_MODE
    {
        INTERACTIVE_MODE,
        RENDER_MODE,
    };

signals:

private:
    AppEnums(const AppEnums& _other) = delete;
    void operator =(const AppEnums& _other) = delete;

};

#endif // APPENUMS_H
