#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <vector>
#include <ctime>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "odometry/feature.h"
#include "odometry/utils.h"
#include "odometry/evaluate_odometry.h"
#include "odometry/visualOdometry.h"

int main(int argc, char **argv){
    std::cout << "Hello VO_Test" << std::endl;

    // -----------------------------------------
    // Load images and calibration parameters
    // -----------------------------------------
    bool display_ground_truth = false;

    std::vector<Matrix> pose_matrix_gt;
    display_ground_truth = true;
    std::cerr << "Display ground truth trajectory" << std::endl;
    // load ground truth pose
    std::string filename_pose = std::string("/home/lacie/Github/Data/Dataset/Kitti/Gray/00/00.txt");
    pose_matrix_gt = loadPoses(filename_pose);


//    if(argc < 3)
//    {
//        std::cerr << "Usage: ./test_vo path_to_sequence path_to_calibration path_to_ground_truth_pose" << std::endl;
//        return 1;
//    }

    // Sequence
    std::string filepath = std::string("/home/lacie/Github/Data/Dataset/Kitti/Gray/00/");
    std::cout << "Filepath: " << filepath << std::endl;

    // Camera calibration
    std::string strSettingPath = std::string("/home/lacie/Github/AG-Mapping/SLAM/config/kitti00.yaml");
    std::cout << "Calibration Filepath: " << strSettingPath << std::endl;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float bf = fSettings["Camera.bf"];

    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);
    std::cout << "P_left: " << std::endl << projMatrl << std::endl;
    std::cout << "P_right: " << std::endl << projMatrr << std::endl;

    // -----------------------------------------
    // Initialize variables
    // -----------------------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat frame_pose32 = cv::Mat::eye(4, 4, CV_32F);

    std::cout << "frame_pose " << frame_pose << std::endl;
    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    FeatureSet currentVOFeatures;
    cv::Mat points4D, points3D;
    int init_frame_id = 0;

    // ------------------------
    // Load first images
    // ------------------------
    cv::Mat imageRight_t0,  imageLeft_t0;

    cv::Mat imageLeft_t0_color;
    loadImageLeft(imageLeft_t0_color,  imageLeft_t0, init_frame_id, filepath);

    cv::Mat imageRight_t0_color;
    loadImageRight(imageRight_t0_color, imageRight_t0, init_frame_id, filepath);

    clock_t t_a, t_b;

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------
    std::vector<FeaturePoint> oldFeaturePointsLeft;
    std::vector<FeaturePoint> currentFeaturePointsLeft;

    for (int frame_id = init_frame_id+1; frame_id < 9000; frame_id++)
    {

        std::cout << std::endl << "frame id " << frame_id << std::endl;
        // ------------
        // Load images
        // ------------
        cv::Mat imageRight_t1,  imageLeft_t1;

        cv::Mat imageLeft_t1_color;
        loadImageLeft(imageLeft_t1_color,  imageLeft_t1, frame_id, filepath);
        cv::Mat imageRight_t1_color;
        loadImageRight(imageRight_t1_color, imageRight_t1, frame_id, filepath);

        t_a = clock();
        std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;

        std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
        matchingFeatures( imageLeft_t0, imageRight_t0,
                          imageLeft_t1, imageRight_t1,
                          currentVOFeatures,
                          pointsLeft_t0,
                          pointsRight_t0,
                          pointsLeft_t1,
                          pointsRight_t1);

        imageLeft_t0 = imageLeft_t1;
        imageRight_t0 = imageRight_t1;

        std::vector<cv::Point2f>& currentPointsLeft_t0 = pointsLeft_t0;
        std::vector<cv::Point2f>& currentPointsLeft_t1 = pointsLeft_t1;

        std::vector<cv::Point2f> newPoints;
        std::vector<bool> valid; // valid new points are ture

        // ---------------------
        // Triangulate 3D Points
        // ---------------------
        cv::Mat points3D_t0, points4D_t0;
        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t0,  pointsRight_t0,  points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

//        cv::Mat points3D_t1, points4D_t1;
//        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t1,  pointsRight_t1,  points4D_t1);
//        cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);

        // ---------------------
        // Tracking transfomation
        // ---------------------
        clock_t tic_gpu = clock();
        trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation, false);
        clock_t toc_gpu = clock();
        std::cerr << "tracking frame 2 frame: " << float(toc_gpu - tic_gpu)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;
        displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);


//        points4D = points4D_t0;
//        frame_pose.convertTo(frame_pose32, CV_32F);
//        points4D = frame_pose32 * points4D;
//        cv::convertPointsFromHomogeneous(points4D.t(), points3D);

        // ------------------------------------------------
        // Intergrating and display
        // ------------------------------------------------

        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

        cv::Mat rigid_body_transformation;

        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation);

        } else {

            std::cout << "Too large rotation"  << std::endl;
        }
        t_b = clock();
        float frame_time = 1000*(double)(t_b-t_a)/CLOCKS_PER_SEC;
        float fps = 1000/frame_time;
        std::cout << "[Info] frame times (ms): " << frame_time << std::endl;
        std::cout << "[Info] FPS: " << fps << std::endl;

        // std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;
        // std::cout << "rotation: " << rotation_euler << std::endl;
        // std::cout << "translation: " << translation.t() << std::endl;
        // std::cout << "frame_pose" << frame_pose << std::endl;

        cv::Mat xyz = frame_pose.col(3).clone();
        display(frame_id, trajectory, xyz, pose_matrix_gt, fps, display_ground_truth);

    }

    return 0;
}
