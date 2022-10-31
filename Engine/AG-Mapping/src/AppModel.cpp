#include "AppModel.h"
#include "AppConstants.h"

#include <QDebug>

AppModel* AppModel::m_instance = nullptr;
QMutex AppModel::m_lock;
AppEnums::APP_STATE AppModel::m_state = AppEnums::APP_STATE::NONE_STATE;
AppEnums::VSLAM_TYPE AppModel::m_slam_type = AppEnums::VSLAM_TYPE::NONE_TYPE;
AppEnums::SLAM_STATE AppModel::m_slam_state = AppEnums::SLAM_STATE::VSLAM_NONE_STATE;

AppModel::AppModel(QObject *parent)
    : QObject{parent}
{
    CONSOLE << "Init instance";
}

void AppModel::set_slam_type(AppEnums::VSLAM_TYPE type)
{
    m_slam_type = type;
}

AppEnums::VSLAM_TYPE AppModel::get_slam_type()
{
    return m_slam_type;
}

AppModel *AppModel::instance(){
    m_lock.lock();
    if (nullptr == m_instance){
        m_instance = new AppModel();
    }
    m_lock.unlock();
    return m_instance;
}

