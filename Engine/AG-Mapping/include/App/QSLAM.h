#ifndef QSLAM_H
#define QSLAM_H

#include <QObject>
#include <QString>
#include <QThread>
#include <QMutex>
#include <QStringList>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "estimator.h"

class QSLAM : public QObject
{
    Q_OBJECT
public:
    QSLAM();

    void init(QString settingPaths);

    void run(QStringList dataPath);

signals:
    void slamComplete();
};

void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
                vector<string> &strImagesFileNames, vector<double> &timeStamps);
void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp);

// Callback function
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
void img0_callback(const cv::Mat &img_msg, const double &t);
void img1_callback(const cv::Mat &img_msg, const double &t);

void display2D(int frame_id, const Estimator &estimator, cv::Mat& visual);

void runSLAM(QStringList dataPath, QSLAM *m_slam);

void sync_process();

#endif // QSLAM_H
