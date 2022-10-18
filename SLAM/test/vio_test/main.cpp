#include <iostream>

#include "parameters.h"
#include "estimator.h"
#include "tools.h"

// static FeatureTracker trackerData[NUM_OF_CAM];
static Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<pair<cv::Mat, double>> img0_buf;
queue<pair<cv::Mat, double>> img1_buf;

cv::Mat visual = cv::Mat::zeros(600, 1200, CV_8UC3);

std::mutex m_buf;

void img0_callback(const cv::Mat &img_msg, const double &t)
{
    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img0_buf.push(tmp);
    m_buf.unlock();
}

void img1_callback(const cv::Mat &img_msg, const double &t)
{
    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img1_buf.push(tmp);
    m_buf.unlock();
}

void display2D (int frame_id, const Estimator &estimator, cv::Mat& visual){
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

// extract images with same timestamp from two topics
void sync_process()
{
    int frame_id = 0;
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front().second;
                double time1 = img1_buf.front().second;
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front().second;
                    image0 = img0_buf.front().first;
                    img0_buf.pop();
                    image1 = img1_buf.front().first;
                    img1_buf.pop();
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front().second;
                image = img0_buf.front().first;
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        display2D(frame_id, estimator, visual);
        frame_id++;

        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
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

void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
                vector<string> &strImagesFileNames, vector<double> &timeStamps)
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
/******************* load image end ***********************/

/******************* load IMU begin ***********************/

void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp)
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

int main(int argc, char **argv)
{
    /******************* load image begin ***********************/
    //read parameters section
    readParameters("/home/lacie/Github/AG-Mapping/SLAM/config/stereo.yaml");
    estimator.setParameter();

    if (!STEREO)
    {
        //imu data file
        ifstream fImus;
        fImus.open("/home/lacie/Github/Dataset/MH_03_medium/mav0/imu0/data.csv");

        cv::Mat image;
        int ni;//num image

        vector<string> vStrImagesFileNames;
        vector<double> vTimeStamps;
        LoadImages(string("/home/lacie/Github/Dataset/MH_03_medium/mav0/cam0/data"),
                   string("/home/lacie/Github/AG-Mapping/data/euroc_data_timestamp/MH03.txt"),vStrImagesFileNames,vTimeStamps);

        int imageNum = vStrImagesFileNames.size();

        if(imageNum<=0)
        {
            cerr << "ERROR: Failed to load images" << endl;
            return 1;
        }

        std::thread measurement_process{sync_process};

        measurement_process.detach();

        for(ni=0; ni<imageNum; ni++)
        {

            double  tframe = vTimeStamps[ni];   //timestamp
            uint32_t  sec = tframe;
            uint32_t nsec = (tframe-sec)*1e9;
            nsec = (nsec/1000)*1000+500;
            ros::Time image_timestamp = ros::Time(sec, nsec);
            // read imu data
            LoadImus(fImus,image_timestamp);

            //read image from file
            image = cv::imread(vStrImagesFileNames[ni],cv::IMREAD_UNCHANGED);

            if(image.empty())
            {
                cerr << endl << "Failed to load image: " << vStrImagesFileNames[ni] <<endl;
                return 1;
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            img0_callback(image, image_timestamp.toSec());
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double timeSpent =std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();

            //wait to load the next frame image
            double T=0;
            if(ni < imageNum-1)
                T = vTimeStamps[ni+1]-tframe; //interval time between two consecutive frames,unit:second
            else if(ni>0)    //lastest frame
                T = tframe-vTimeStamps[ni-1];

            if(timeSpent < T)
                usleep((T-timeSpent)*1e6); //sec->us:1e6
            else
                cerr << endl << "process image speed too slow, larger than interval time between two consecutive frames" << endl;

        }
    }
    else if(STEREO)
    {
        //imu data file
        ifstream fImus;
        fImus.open("/home/lacie/Github/Dataset/MH_03_medium/mav0/imu0/data.csv"); // check

        cv::Mat image;
        cv::Mat image2;
        int ni;//num image

        vector<string> vStrImagesFileNames;
        vector<string> vStrImagesFileNames2 ;
        vector<double> vTimeStamps;
        vector<double> vTimeStamps2;
        LoadImages(string("/home/lacie/Github/Dataset/MH_03_medium/mav0/cam0/data"),string("/home/lacie/Github/AG-Mapping/data/euroc_data_timestamp/MH03.txt"),vStrImagesFileNames,vTimeStamps); //left
        LoadImages(string("/home/lacie/Github/Dataset/MH_03_medium/mav0/cam1/data"),string("/home/lacie/Github/AG-Mapping/data/euroc_data_timestamp/MH03.txt"),vStrImagesFileNames2,vTimeStamps2); //right

        int tmp_imageNum = vStrImagesFileNames.size();
        int tmp_imageNum2 = vStrImagesFileNames2.size();
        int imageNum = (tmp_imageNum<tmp_imageNum2)?tmp_imageNum2:tmp_imageNum; // I am good at coding. hahaha


        if(imageNum<=0)
        {
            cerr << "ERROR: Failed to load images" << endl;
            return 1;
        }

        std::thread measurement_process{sync_process};

        measurement_process.detach();

        for(ni=0; ni<imageNum; ni++)
        {

            double  tframe = vTimeStamps[ni];   //timestamp
            uint32_t  sec = tframe;
            uint32_t nsec = (tframe-sec)*1e9;
            nsec = (nsec/1000)*1000+500;
            ros::Time image_timestamp = ros::Time(sec, nsec);
            double  tframe2 = vTimeStamps2[ni];   //timestamp
            uint32_t  sec2 = tframe2;
            uint32_t nsec2 = (tframe2-sec2)*1e9;
            nsec2 = (nsec2/1000)*1000+500;
            ros::Time image_timestamp2 = ros::Time(sec2, nsec2);
            // read imu data
            LoadImus(fImus,image_timestamp); //TODO

            //read image from file
            image = cv::imread(vStrImagesFileNames[ni],cv::IMREAD_UNCHANGED);
            image2 = cv::imread(vStrImagesFileNames2[ni],cv::IMREAD_UNCHANGED);

            if(image.empty() or image2.empty())
            {
                cerr << endl << "Failed to load image: " << vStrImagesFileNames[ni] <<endl;
                return 1;
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            img0_callback(image, image_timestamp.toSec());
            img1_callback(image2, image_timestamp2.toSec());
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double timeSpent =std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();

            //wait to load the next frame image
            double T=0;
            if(ni < imageNum-1)
                T = vTimeStamps[ni+1]-tframe; //interval time between two consecutive frames,unit:second
            else if(ni>0)    //lastest frame
                T = tframe-vTimeStamps[ni-1];

            if(timeSpent < T)
                usleep((T-timeSpent)*1e6); //sec->us:1e6
            else
                cerr << endl << "process image speed too slow, larger than interval time between two consecutive frames" << endl;
        }

        return 0;
    }
}