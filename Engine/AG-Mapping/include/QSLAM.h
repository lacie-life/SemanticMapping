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
    QSLAM(QObject *parent = nullptr, QString settingPaths = "");

    void run(QStringList dataPath);

    void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
                    vector<string> &strImagesFileNames, vector<double> &timeStamps);
    void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp);

    // Callback function
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void img0_callback(const cv::Mat &img_msg, const double &t);
    void img1_callback(const cv::Mat &img_msg, const double &t);

    void display2D(int frame_id, const Estimator &estimator, cv::Mat& visual);

signals:

public:
    static Estimator estimator;
    QMutex *m_mutex;

    queue<sensor_msgs::ImuConstPtr> imu_buf;
    queue<sensor_msgs::PointCloudConstPtr> feature_buf;
    queue<pair<cv::Mat, double>> img0_buf;
    queue<pair<cv::Mat, double>> img1_buf;

    cv::Mat visual = cv::Mat::zeros(600, 1200, CV_8UC3);

};

#endif // QSLAM_H
