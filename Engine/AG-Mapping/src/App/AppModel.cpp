#include "AppModel.h"
#include "AppConstants.h"

#include <QDebug>
#include <QStringList>

AppModel* AppModel::m_instance = nullptr;
QMutex AppModel::m_lock;
AppEnums::APP_STATE AppModel::m_state = AppEnums::APP_STATE::NONE_STATE;
AppEnums::VSLAM_TYPE AppModel::m_slam_type = AppEnums::VSLAM_TYPE::NONE_TYPE;
AppEnums::SLAM_STATE AppModel::m_slam_state = AppEnums::SLAM_STATE::VSLAM_NONE_STATE;

AppModel::AppModel(QObject *parent)
    : QObject{parent}
{
    CONSOLE << "Init instance";

    m_slam = new QSLAM();

    m_slamSettingPath = PARAM_PATH;
    m_IMUPath = QString(IMAGE_PATH) + "/imu0/data.csv";
    m_LeftImgPath = QString(IMAGE_PATH) + "/cam0/data";
    m_RightImgPath = QString(IMAGE_PATH) + "/cam1/data";
    m_AssoPath = QString(IMAGE_PATH) + "/timestamp/data/time.txt";

    connect(m_slam, &QSLAM::slamComplete, this, []()
    {
        CONSOLE << "SLAM Completed !!";
    });

    connect(m_slam, &QSLAM::trajectoryUpdate, m_slam, [this](QImage img)
    {
        CONSOLE << "Test ?";
        emit this->updateTrajactory(img);
    });
}

void AppModel::set_slam_type(AppEnums::VSLAM_TYPE type)
{
    m_slam_type = type;
}

AppEnums::VSLAM_TYPE AppModel::get_slam_type()
{
    return m_slam_type;
}

void AppModel::SLAM_Run()
{
    m_slam->init(m_slamSettingPath);

    QStringList temp;
    temp.append(m_IMUPath);
    temp.append(m_LeftImgPath);
    temp.append(m_RightImgPath);
    temp.append(m_AssoPath);

    m_slam->run(temp);
}

AppModel *AppModel::instance(){
    m_lock.lock();
    if (nullptr == m_instance){
        m_instance = new AppModel();
    }
    m_lock.unlock();
    return m_instance;
}

