//
// Created by lacie on 10/10/2022.
//

#ifndef SEMANTICSLAM_SOLVE_5PTS_H
#define SEMANTICSLAM_SOLVE_5PTS_H

#include <vector>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class MotionEstimator
{
public:

    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

private:
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};

#endif //SEMANTICSLAM_SOLVE_5PTS_H
