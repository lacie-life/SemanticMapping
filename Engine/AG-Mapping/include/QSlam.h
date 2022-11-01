#ifndef QSLAM_H
#define QSLAM_H

#include <QObject>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <memory>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "estimator.h"
#include "parameters.h"
#include "AppModel.h"

class QSLAM
{
public:
    explicit QSLAM(QObject *parent = nullptr);

    void run();

    void shutdown();

    std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void img0_callback(const cv::Mat &img_msg, const double &t);
    void img1_callback(const cv::Mat &img_msg, const double &t);
    void sync_process();
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
                    vector<string> &strImagesFileNames, vector<double> &timeStamps);
    void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp);

private:
    static Estimator m_estimator;
    QMutex *m_mutex;

    queue<sensor_msgs::ImuConstPtr> imu_buf;
    queue<sensor_msgs::PointCloudConstPtr> feature_buf;
    queue<pair<cv::Mat, double>> img0_buf;
    queue<pair<cv::Mat, double>> img1_buf;

};

#endif // QSLAM_H
