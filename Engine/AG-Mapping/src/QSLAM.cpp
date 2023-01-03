#include "QSLAM.h"

QSLAM::QSLAM(QObject *parent, QString settingPaths)
    : QObject{parent}
{
    readParameters(settingPaths.toStdString());
    estimator.setParameter();
}

void QSLAM::LoadImages(const string &strImagePath, const string &strTimesStampsPath, vector<string> &strImagesFileNames, vector<double> &timeStamps)
{
    ifstream fTimes;
    fTimes.open(strTimesStampsPath.c_str());
    timeStamps.reserve(5000); //reserve vector space
    strImagesFileNames.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            strImagesFileNames.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            timeStamps.push_back(t/1e9);
        }
    }

}

void QSLAM::LoadImus(ifstream &fImus, const ros::Time &imageTimestamp)
{
    while(!fImus.eof())
    {
        string s;
        getline(fImus,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')      //remove first line in data.csv
                continue;
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt=0;
            double data[7];
            while(ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if(cnt ==7)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            data[0] *=1e-9; //convert to second unit
            sensor_msgs::ImuPtr imudata(new sensor_msgs::Imu);
            imudata->angular_velocity.x = data[1];
            imudata->angular_velocity.y = data[2];
            imudata->angular_velocity.z = data[3];
            imudata->linear_acceleration.x = data[4];
            imudata->linear_acceleration.y = data[5];
            imudata->linear_acceleration.z = data[6];
            uint32_t  sec = data[0];
            uint32_t nsec = (data[0]-sec)*1e9;
            nsec = (nsec/1000)*1000+500;
            imudata->header.stamp = ros::Time(sec,nsec);
            imu_callback(imudata);
            if (imudata->header.stamp > imageTimestamp)       //load all imu data produced in interval time between two consecutive frams
                break;
        }
    }
}

void QSLAM::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

void QSLAM::img0_callback(const cv::Mat &img_msg, const double &t)
{
    m_mutex->lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img0_buf.push(tmp);
    m_mutex->unlock();
}

void QSLAM::img1_callback(const cv::Mat &img_msg, const double &t)
{
    m_mutex->lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img1_buf.push(tmp);
    m_mutex->unlock();
}

void QSLAM::display2D(int frame_id, const Estimator &estimator, cv::Mat &visual)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        nav_msgs::Odometry odometry;

        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();

        // draw estimated trajectory
        int x = int(odometry.pose.pose.position.x) + 300;
        int y = int(odometry.pose.pose.position.y) + 100;
        circle(visual, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

        cv::imshow( "Trajectory", visual);

        cv::waitKey(1);
    }
}
