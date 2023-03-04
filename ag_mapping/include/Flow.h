//
// Created by lacie on 28/01/2023.
//

#ifndef AG_MAPPING_FLOW_H
#define AG_MAPPING_FLOW_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Frame.h"

using namespace std;

namespace AG_MAPPING
{
    class Flow {
    private:
        cv::Mat mImGrayLast;
        cv::Mat mImGrayCurrent;
        //cv::Mat mImSource;
        // float mBInaryThreshold;

    public:
        Flow();
        ~Flow() = default;
        void ComputeMask(const cv::Mat& GrayImg, cv::Mat& mask, float BInaryThreshold);
        void ComputeMask(const cv::Mat& GrayImg, const cv::Mat& Homo, cv::Mat& mask, float BInaryThreshold);
    private:
        // Pass in the homography transformation matrix
        void myWarpPerspective(const cv::Mat& src, cv::Mat& dst, const cv::Mat& H);
    };
}

#endif //AG_MAPPING_FLOW_H
