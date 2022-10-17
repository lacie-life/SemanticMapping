#ifndef SEMANTICSLAM_PARAMETERS_H
#define SEMANTICSLAM_PARAMETERS_H

#include "ros_things.h"
#include "camodocal/camera_models.h"

#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <cassert>
#include <cstring>
#include <csignal>
#include <map>
#include <unordered_map>
#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <deque>
#include <queue>
#include <thread>
#include <mutex>

#include <assert.h>
#include <unistd.h>
#include <string>

#include <pthread.h>
#include <execinfo.h>
#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#ifdef USE_CUDA
#include <opencv2/cudaoptflow.hpp> // GPU
#include <opencv2/cudaimgproc.hpp> // GPU
#include <opencv2/cudaarithm.hpp>  // GPU
#endif

#include <Eigen/Dense>

#include <iostream> // error
#include <fstream>

using namespace std;
using namespace camodocal;
using namespace Eigen;

// parameters //TODO
const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
extern int NUM_OF_CAM;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
extern int USE_GPU;
extern int USE_GPU_ACC_FLOW;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string FISHEYE_MASK;
extern cv::Mat fisheye_mask;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int SHOW_TMI; // added
extern int FISHEYE;  // added

void readParameters(const string &config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

#endif //SEMANTICSLAM_PARAMETERS_H
