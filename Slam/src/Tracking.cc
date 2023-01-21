/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"
#include "ros_moc.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

// Semantic path
#include "PointCloudMapping.h"
#include "MapObject.h"
#include "Config.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include <algorithm>
#include <boost/filesystem.hpp>
#include <ctime>
#include <math.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

#include <mutex>
#include <chrono>


using namespace std;

namespace ORB_SLAM3 {

/**
 * @brief Construct a new Tracking:: Tracking object
 *
 * @param pSys 
 * @param pVoc 
 * @param pFrameDrawer 
 * @param pMapDrawer 
 * @param pAtlas 
 * @param pKFDB 
 * @param strSettingPath 
 * @param sensor 
 * @param settings 
 * @param _nameSeq 
 */
    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                       Atlas *pAtlas,
                       shared_ptr<PointCloudMapping> pPointCloud, KeyFrameDatabase *pKFDB, const string &strSettingPath,
                       const int sensor, Settings *settings,
                       const string &_nameSeq) :
            mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
            mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
            mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
            mpPointCloudMapping(pPointCloud),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0),
            time_recently_lost(5.0),
            mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr),
            mpLastKeyFrame(static_cast<KeyFrame *>(NULL)) {


        // Load camera parameters from settings file
        if (settings) {
            newParameterLoader(settings);
        } else {
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool b_parse_cam = ParseCamParamFile(fSettings);
            if (!b_parse_cam) {
                std::cout << "*Error with the camera parameters in the config file*" << std::endl;
            }

            // Load ORB parameters
            bool b_parse_orb = ParseORBParamFile(fSettings);
            if (!b_parse_orb) {
                std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
            }

            bool b_parse_imu = true;
            if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO || sensor == System::IMU_RGBD) {
                b_parse_imu = ParseIMUParamFile(fSettings);
                if (!b_parse_imu) {
                    std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
                }

                mnFramesToResetIMU = mMaxFrames;
            }

            if (!b_parse_cam || !b_parse_orb || !b_parse_imu) {
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try {
                    throw -1;
                }
                catch (exception &e) {

                }
            }
        }

        initID = 0;
        lastID = 0;
        mbInitWith3KFs = false;
        mnNumDataset = 0;

        vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
        std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
        for (GeometricCamera *pCam: vpCams) {
            std::cout << "Camera " << pCam->GetId();
            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
                std::cout << " is pinhole" << std::endl;
            } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
                std::cout << " is fisheye" << std::endl;
            } else {
                std::cout << " is unknown" << std::endl;
            }
        }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
    }

#ifdef REGISTER_TIMES
    double calcAverage(vector<double> v_times)
    {
        double accum = 0;
        for(double value : v_times)
        {
            accum += value;
        }

        return accum / v_times.size();
    }

    double calcDeviation(vector<double> v_times, double average)
    {
        double accum = 0;
        for(double value : v_times)
        {
            accum += pow(value - average, 2);
        }
        return sqrt(accum / v_times.size());
    }

    double calcAverage(vector<int> v_values)
    {
        double accum = 0;
        int total = 0;
        for(double value : v_values)
        {
            if(value == 0)
                continue;
            accum += value;
            total++;
        }

        return accum / total;
    }

    double calcDeviation(vector<int> v_values, double average)
    {
        double accum = 0;
        int total = 0;
        for(double value : v_values)
        {
            if(value == 0)
                continue;
            accum += pow(value - average, 2);
            total++;
        }
        return sqrt(accum / total);
    }

    void Tracking::LocalMapStats2File()
    {
        ofstream f;
        f.open("LocalMapTimeStats.txt");
        f << fixed << setprecision(6);
        f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
        for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
        {
            f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
            << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
            << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
        }

        f.close();

        f.open("LBA_Stats.txt");
        f << fixed << setprecision(6);
        f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
        for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
        {
            f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
            << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
            << mpLocalMapper->vnLBA_edges[i] << endl;
        }

        f.close();
    }

    void Tracking::TrackStats2File()
    {
        ofstream f;
        f.open("SessionInfo.txt");
        f << fixed;
        f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
        f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

        f << "OpenCV version: " << CV_VERSION << endl;

        f.close();

        f.open("TrackingTimeStats.txt");
        f << fixed << setprecision(6);

        f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

        for(int i=0; i<vdTrackTotal_ms.size(); ++i)
        {
            double stereo_rect = 0.0;
            if(!vdRectStereo_ms.empty())
            {
                stereo_rect = vdRectStereo_ms[i];
            }

            double resize_image = 0.0;
            if(!vdResizeImage_ms.empty())
            {
                resize_image = vdResizeImage_ms[i];
            }

            double stereo_match = 0.0;
            if(!vdStereoMatch_ms.empty())
            {
                stereo_match = vdStereoMatch_ms[i];
            }

            double imu_preint = 0.0;
            if(!vdIMUInteg_ms.empty())
            {
                imu_preint = vdIMUInteg_ms[i];
            }

            f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
            << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
        }

        f.close();
    }

    void Tracking::PrintTimeStats()
    {
        // Save data in files
        TrackStats2File();
        LocalMapStats2File();


        ofstream f;
        f.open("ExecMean.txt");
        f << fixed;
        //Report the mean and std of each one
        std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
        f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
        cout << "OpenCV version: " << CV_VERSION << endl;
        f << "OpenCV version: " << CV_VERSION << endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
        f << "---------------------------" << std::endl;
        f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
        double average, deviation;
        if(!vdRectStereo_ms.empty())
        {
            average = calcAverage(vdRectStereo_ms);
            deviation = calcDeviation(vdRectStereo_ms, average);
            std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
            f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        }

        if(!vdResizeImage_ms.empty())
        {
            average = calcAverage(vdResizeImage_ms);
            deviation = calcDeviation(vdResizeImage_ms, average);
            std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
            f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        }

        average = calcAverage(vdORBExtract_ms);
        deviation = calcDeviation(vdORBExtract_ms, average);
        std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
        f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

        if(!vdStereoMatch_ms.empty())
        {
            average = calcAverage(vdStereoMatch_ms);
            deviation = calcDeviation(vdStereoMatch_ms, average);
            std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
            f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        }

        if(!vdIMUInteg_ms.empty())
        {
            average = calcAverage(vdIMUInteg_ms);
            deviation = calcDeviation(vdIMUInteg_ms, average);
            std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
            f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        }

        average = calcAverage(vdPosePred_ms);
        deviation = calcDeviation(vdPosePred_ms, average);
        std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
        f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdLMTrack_ms);
        deviation = calcDeviation(vdLMTrack_ms, average);
        std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
        f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdNewKF_ms);
        deviation = calcDeviation(vdNewKF_ms, average);
        std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
        f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(vdTrackTotal_ms);
        deviation = calcDeviation(vdTrackTotal_ms, average);
        std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

        // Local Mapping time stats
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Local Mapping" << std::endl << std::endl;
        f << std::endl << "Local Mapping" << std::endl << std::endl;

        average = calcAverage(mpLocalMapper->vdKFInsert_ms);
        deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
        std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
        f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdMPCulling_ms);
        deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
        std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
        f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdMPCreation_ms);
        deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
        std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
        f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdLBA_ms);
        deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
        std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdKFCulling_ms);
        deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
        std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
        f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vdLMTotal_ms);
        deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
        std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

        // Local Mapping LBA complexity
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
        f << "---------------------------" << std::endl;
        f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_edges);
        deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
        std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_KFopt);
        deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
        std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
        deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
        std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
        f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

        average = calcAverage(mpLocalMapper->vnLBA_MPs);
        deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
        std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
        f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

        std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
        std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
        f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
        f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

        // Map complexity
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl << "Map complexity" << std::endl;
        std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
        std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
        f << "---------------------------" << std::endl;
        f << std::endl << "Map complexity" << std::endl;
        vector<Map*> vpMaps = mpAtlas->GetAllMaps();
        Map* pBestMap = vpMaps[0];
        for(int i=1; i<vpMaps.size(); ++i)
        {
            if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
            {
                pBestMap = vpMaps[i];
            }
        }

        f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
        f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

        f << "---------------------------" << std::endl;
        f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdDataQuery_ms);
        deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
        f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdEstSim3_ms);
        deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
        f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdPRTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
        f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
        std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

        f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
        std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
        f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
        f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
        f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
        std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

        f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
        average = calcAverage(mpLoopClosing->vnLoopKFs);
        deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

        f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
        std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
        f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
        deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
        f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
        f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
        f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
        std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

        f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
        average = calcAverage(mpLoopClosing->vnMergeKFs);
        deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vnMergeMPs);
        deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
        f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

        f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
        std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
        average = calcAverage(mpLoopClosing->vdGBA_ms);
        deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
        f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
        deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
        f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
        deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
        f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
        std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

        f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
        std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
        f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
        std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
        average = calcAverage(mpLoopClosing->vnGBAKFs);
        deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
        f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vnGBAMPs);
        deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
        f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
        std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

        f.close();

    }

#endif

    Tracking::~Tracking() {
        //f_track_stats.close();

    }

    void Tracking::newParameterLoader(Settings *settings) {
        mpCamera = settings->camera1();
        mpCamera = mpAtlas->AddCamera(mpCamera);

        if (settings->needToUndistort()) {
            mDistCoef = settings->camera1DistortionCoef();
        } else {
            mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
        }

        //TODO: missing image scaling and rectification
        mImageScale = 1.0f;

        mK = cv::Mat::eye(3, 3, CV_32F);
        mK.at<float>(0, 0) = mpCamera->getParameter(0);
        mK.at<float>(1, 1) = mpCamera->getParameter(1);
        mK.at<float>(0, 2) = mpCamera->getParameter(2);
        mK.at<float>(1, 2) = mpCamera->getParameter(3);

        mK_.setIdentity();
        mK_(0, 0) = mpCamera->getParameter(0);
        mK_(1, 1) = mpCamera->getParameter(1);
        mK_(0, 2) = mpCamera->getParameter(2);
        mK_(1, 2) = mpCamera->getParameter(3);

        if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            settings->cameraType() == Settings::KannalaBrandt) {
            mpCamera2 = settings->camera2();
            mpCamera2 = mpAtlas->AddCamera(mpCamera2);

            mTlr = settings->Tlr();

            mpFrameDrawer->both = true;
        }

        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD) {
            mbf = settings->bf();
            mThDepth = settings->b() * settings->thDepth();
        }

        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
            mDepthMapFactor = settings->depthMapFactor();
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

        mMinFrames = 0;
        mMaxFrames = settings->fps();
        mbRGB = settings->rgb();

        //ORB parameters
        int nFeatures = settings->nFeatures();
        int nLevels = settings->nLevels();
        int fIniThFAST = settings->initThFAST();
        int fMinThFAST = settings->minThFAST();
        float fScaleFactor = settings->scaleFactor();

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        //IMU parameters
        Sophus::SE3f Tbc = settings->Tbc();
        mInsertKFsLost = settings->insertKFsWhenLost();
        mImuFreq = settings->imuFrequency();
        mImuPer = 0.001; //1.0 / (double) mImuFreq;
        float Ng = settings->noiseGyro();
        float Na = settings->noiseAcc();
        float Ngw = settings->gyroWalk();
        float Naw = settings->accWalk();

        const float sf = sqrt(mImuFreq);
        mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    bool Tracking::ParseCUBOIDParamFile(cv::FileStorage &fSettings) {
        InitToGround = cv::Mat::eye(4, 4, CV_32F);
        // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
        double init_x = ORB_SLAM3::otherSettings["init_x"];
        double init_y = ORB_SLAM3::otherSettings["init_y"];
        double init_z = ORB_SLAM3::otherSettings["init_z"];
        double init_qx = ORB_SLAM3::otherSettings["init_qx"];
        double init_qy = ORB_SLAM3::otherSettings["init_qy"];
        double init_qz = ORB_SLAM3::otherSettings["init_qz"];
        double init_qw = ORB_SLAM3::otherSettings["init_qw"];

        Eigen::Quaternionf pose_quat(init_qw, init_qx, init_qy, init_qz);
        Eigen::Matrix3f rot =
                pose_quat.toRotationMatrix(); // The quaternion is required to be normalized
        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                InitToGround.at<float>(row, col) = rot(row, col);

        InitToGround.at<float>(0, 3) = init_x;
        InitToGround.at<float>(1, 3) = init_y;
        InitToGround.at<float>(2, 3) = init_z;
        nominal_ground_height = init_z;

        cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = InitToGround.rowRange(0, 3).col(3);
        cv::Mat Rinv = R.t();
        cv::Mat Ow = -Rinv * t;
        GroundToInit = cv::Mat::eye(4, 4, CV_32F);
        Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));

        InitToGround_eigen = Converter::toMatrix4f(InitToGround);
        GroundToInit_eigen = Converter::toMatrix4f(GroundToInit);

        mpMap->InitToGround = InitToGround;
        mpMap->GroundToInit = GroundToInit.clone();
        mpMap->InitToGround_eigen = InitToGround_eigen;
        mpMap->InitToGround_eigen_d = InitToGround_eigen.cast<double>();
        mpMap->GroundToInit_eigen_d = GroundToInit_eigen.cast<double>();
        mpMap->GroundToInit_opti = GroundToInit.clone();
        mpMap->InitToGround_opti = InitToGround.clone();
        mpMap->RealGroundToMine_opti = cv::Mat::eye(4, 4, CV_32F);
        mpMap->MineGroundToReal_opti = cv::Mat::eye(4, 4, CV_32F);

        float fx = ORB_SLAM3::otherSettings["Camera.fx"];
        float fy = ORB_SLAM3::otherSettings["Camera.fy"];
        float cx = ORB_SLAM3::otherSettings["Camera.cx"];
        float cy = ORB_SLAM3::otherSettings["Camera.cy"];

        Kalib.setIdentity();
        Kalib(0, 0) = fx;
        Kalib(0, 2) = cx;
        Kalib(1, 1) = fy;
        Kalib(1, 2) = cy;
        Kalib_f = Kalib.cast<float>();
        invKalib = Kalib.inverse();
        invKalib_f = Kalib_f.inverse();
        mpMap->Kalib = Kalib;
        mpMap->Kalib_f = Kalib_f;
        mpMap->invKalib_f = invKalib_f;

        obj_det_2d_thre = ORB_SLAM3::otherSettings["obj_det_2d_thre"];
        // transform initial pose and map to ground frame
        build_worldframe_on_ground = ORB_SLAM3::otherSettings["build_worldframe_on_ground"];
        triangulate_dynamic_pts = ORB_SLAM3::otherSettings["triangulate_dynamic_pts"];

        use_truth_trackid = false; // whether use ground truth tracklet ID.
        whether_save_online_detected_cuboids = false;
        whether_save_final_optimized_cuboids = false;
        if (whether_detect_object)
        {
            use_truth_trackid = ORB_SLAM3::otherSettings["use_truth_trackid"];
            if (!whether_read_offline_cuboidtxt)
            {
                detect_cuboid_obj = new detect_3d_cuboid();
                detect_cuboid_obj->print_details = false; // false  true
                detect_cuboid_obj->whether_sample_cam_roll_pitch =
                        ORB_SLAM3::otherSettings["whether_sample_cam_roll_pitch"];
                detect_cuboid_obj->set_calibration(Kalib);
            }

            if (!whether_read_offline_cuboidtxt)
            {
                whether_save_online_detected_cuboids =
                        ORB_SLAM3::otherSettings["whether_save_online_detected_cuboids"];
                if (whether_save_online_detected_cuboids)
                {
                    std::string save_object_pose_txt = data_folder + "/orb_live_pred_objs_temp.txt";
                    save_online_detected_cuboids.open(save_object_pose_txt.c_str());
                }
            }
            if (whether_read_offline_cuboidtxt)
                ReadAllObjecttxt();
        }

        if (whether_detect_object)
        {
            whether_save_final_optimized_cuboids =
                    ORB_SLAM3::otherSettings["whether_save_final_optimized_cuboids"];
            if (whether_save_final_optimized_cuboids)
            {
                if (final_object_record_frame_ind == 1e5)
                {
                    ROS_ERROR_STREAM("Please set final_object_record_frame_ind!!!");
                    whether_save_final_optimized_cuboids = false;
                }
            }
        }

        std::string truth_pose_txts = data_folder + "/pose_truth.txt";
        Eigen::MatrixXd truth_cam_poses(5, 8);
        if (read_all_number_txt(truth_pose_txts, truth_cam_poses))
        {
            mpMapDrawer->truth_poses.resize(truth_cam_poses.rows() / 10, 3);
            for (int i = 0; i < mpMapDrawer->truth_poses.rows(); i++)
            {
                mpMapDrawer->truth_poses.row(i) = truth_cam_poses.row(i * 10).segment(1, 3);
                if (build_worldframe_on_ground)
                {
                    Eigen::Quaterniond pose_quat(truth_cam_poses(i * 10, 7), truth_cam_poses(i * 10, 4),
                                                 truth_cam_poses(i * 10, 5),
                                                 truth_cam_poses(i * 10, 6));
                    Eigen::Matrix4d pose_to_init;
                    pose_to_init.setIdentity();
                    pose_to_init.block(0, 0, 3, 3) = pose_quat.toRotationMatrix();
                    pose_to_init.col(3).head<3>() =
                            Eigen::Vector3d(truth_cam_poses(i * 10, 1), truth_cam_poses(i * 10, 2),
                                            truth_cam_poses(i * 10, 3));
                    Eigen::Matrix4d pose_to_ground = InitToGround_eigen.cast<double>() * pose_to_init;
                    mpMapDrawer->truth_poses.row(i) = pose_to_ground.col(3).head<3>();
                }
            }
            cout << "Read sampled truth pose size for visualization:  "
                 << mpMapDrawer->truth_poses.rows() << endl;
        }

        filtered_ground_height = 0;
        first_absolute_scale_frameid = 0;
        first_absolute_scale_framestamp = 0;
        ground_roi_middle = ORB_SLAM3::otherSettings["ground_roi_middle"];
        ground_roi_lower = ORB_SLAM3::otherSettings["ground_roi_lower"];
        ground_inlier_pts = ORB_SLAM3::otherSettings["ground_inlier_pts"];
        ground_dist_ratio = ORB_SLAM3::otherSettings["ground_dist_ratio"];
        ground_everyKFs = ORB_SLAM3::otherSettings["ground_everyKFs"];
    }

    bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings) {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
        cout << endl << "Camera Parameters: " << endl;
        bool b_miss_params = false;

        string sCameraName = fSettings["Camera.type"];
        if (sCameraName == "PinHole") {
            float fx, fy, cx, cy;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal()) {
                fx = node.real();
            } else {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal()) {
                fy = node.real();
            } else {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal()) {
                cx = node.real();
            } else {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal()) {
                cy = node.real();
            } else {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal()) {
                mDistCoef.at<float>(0) = node.real();
            } else {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal()) {
                mDistCoef.at<float>(1) = node.real();
            } else {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p1"];
            if (!node.empty() && node.isReal()) {
                mDistCoef.at<float>(2) = node.real();
            } else {
                std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.p2"];
            if (!node.empty() && node.isReal()) {
                mDistCoef.at<float>(3) = node.real();
            } else {
                std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal()) {
                mDistCoef.resize(5);
                mDistCoef.at<float>(4) = node.real();
            }

            node = fSettings["Camera.imageScale"];
            if (!node.empty() && node.isReal()) {
                mImageScale = node.real();
            }

            if (b_miss_params) {
                return false;
            }

            if (mImageScale != 1.f) {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx, fy, cx, cy};

            mpCamera = new Pinhole(vCamCalib);

            mpCamera = mpAtlas->AddCamera(mpCamera);

            std::cout << "- Camera: Pinhole" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
            std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


            std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
            std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

            if (mDistCoef.rows == 5)
                std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 2) = cx;
            mK.at<float>(1, 2) = cy;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 2) = cx;
            mK_(1, 2) = cy;
        } else if (sCameraName == "KannalaBrandt8") {
            float fx, fy, cx, cy;
            float k1, k2, k3, k4;
            mImageScale = 1.f;

            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera.fx"];
            if (!node.empty() && node.isReal()) {
                fx = node.real();
            } else {
                std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.fy"];
            if (!node.empty() && node.isReal()) {
                fy = node.real();
            } else {
                std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cx"];
            if (!node.empty() && node.isReal()) {
                cx = node.real();
            } else {
                std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.cy"];
            if (!node.empty() && node.isReal()) {
                cy = node.real();
            } else {
                std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera.k1"];
            if (!node.empty() && node.isReal()) {
                k1 = node.real();
            } else {
                std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera.k2"];
            if (!node.empty() && node.isReal()) {
                k2 = node.real();
            } else {
                std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k3"];
            if (!node.empty() && node.isReal()) {
                k3 = node.real();
            } else {
                std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.k4"];
            if (!node.empty() && node.isReal()) {
                k4 = node.real();
            } else {
                std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera.imageScale"];
            if (!node.empty() && node.isReal()) {
                mImageScale = node.real();
            }

            if (!b_miss_params) {
                if (mImageScale != 1.f) {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;
                }

                vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
                mpCamera = new KannalaBrandt8(vCamCalib);
                mpCamera = mpAtlas->AddCamera(mpCamera);
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                mK = cv::Mat::eye(3, 3, CV_32F);
                mK.at<float>(0, 0) = fx;
                mK.at<float>(1, 1) = fy;
                mK.at<float>(0, 2) = cx;
                mK.at<float>(1, 2) = cy;

                mK_.setIdentity();
                mK_(0, 0) = fx;
                mK_(1, 1) = fy;
                mK_(0, 2) = cx;
                mK_(1, 2) = cy;
            }

            if (mSensor == System::STEREO || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                // Right camera
                // Camera calibration parameters
                cv::FileNode node = fSettings["Camera2.fx"];
                if (!node.empty() && node.isReal()) {
                    fx = node.real();
                } else {
                    std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.fy"];
                if (!node.empty() && node.isReal()) {
                    fy = node.real();
                } else {
                    std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cx"];
                if (!node.empty() && node.isReal()) {
                    cx = node.real();
                } else {
                    std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.cy"];
                if (!node.empty() && node.isReal()) {
                    cy = node.real();
                } else {
                    std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                // Distortion parameters
                node = fSettings["Camera2.k1"];
                if (!node.empty() && node.isReal()) {
                    k1 = node.real();
                } else {
                    std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }
                node = fSettings["Camera2.k2"];
                if (!node.empty() && node.isReal()) {
                    k2 = node.real();
                } else {
                    std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k3"];
                if (!node.empty() && node.isReal()) {
                    k3 = node.real();
                } else {
                    std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }

                node = fSettings["Camera2.k4"];
                if (!node.empty() && node.isReal()) {
                    k4 = node.real();
                } else {
                    std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                    b_miss_params = true;
                }


                int leftLappingBegin = -1;
                int leftLappingEnd = -1;

                int rightLappingBegin = -1;
                int rightLappingEnd = -1;

                node = fSettings["Camera.lappingBegin"];
                if (!node.empty() && node.isInt()) {
                    leftLappingBegin = node.operator int();
                } else {
                    std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera.lappingEnd"];
                if (!node.empty() && node.isInt()) {
                    leftLappingEnd = node.operator int();
                } else {
                    std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingBegin"];
                if (!node.empty() && node.isInt()) {
                    rightLappingBegin = node.operator int();
                } else {
                    std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
                }
                node = fSettings["Camera2.lappingEnd"];
                if (!node.empty() && node.isInt()) {
                    rightLappingEnd = node.operator int();
                } else {
                    std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
                }

                node = fSettings["Tlr"];
                cv::Mat cvTlr;
                if (!node.empty()) {
                    cvTlr = node.mat();
                    if (cvTlr.rows != 3 || cvTlr.cols != 4) {
                        std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                        b_miss_params = true;
                    }
                } else {
                    std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                    b_miss_params = true;
                }

                if (!b_miss_params) {
                    if (mImageScale != 1.f) {
                        // K matrix parameters must be scaled.
                        fx = fx * mImageScale;
                        fy = fy * mImageScale;
                        cx = cx * mImageScale;
                        cy = cy * mImageScale;

                        leftLappingBegin = leftLappingBegin * mImageScale;
                        leftLappingEnd = leftLappingEnd * mImageScale;
                        rightLappingBegin = rightLappingBegin * mImageScale;
                        rightLappingEnd = rightLappingEnd * mImageScale;
                    }

                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                    mpFrameDrawer->both = true;

                    vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                    mpCamera2 = new KannalaBrandt8(vCamCalib2);
                    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                    mTlr = Converter::toSophus(cvTlr);

                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                    static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                    std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                    std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                    std::cout << "- Camera: Fisheye" << std::endl;
                    std::cout << "- Image scale: " << mImageScale << std::endl;
                    std::cout << "- fx: " << fx << std::endl;
                    std::cout << "- fy: " << fy << std::endl;
                    std::cout << "- cx: " << cx << std::endl;
                    std::cout << "- cy: " << cy << std::endl;
                    std::cout << "- k1: " << k1 << std::endl;
                    std::cout << "- k2: " << k2 << std::endl;
                    std::cout << "- k3: " << k3 << std::endl;
                    std::cout << "- k4: " << k4 << std::endl;

                    std::cout << "- mTlr: \n" << cvTlr << std::endl;

                    std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
                }
            }

            if (b_miss_params) {
                return false;
            }

        } else {
            std::cerr << "*Not Supported Camera Sensor*" << std::endl;
            std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
        }

        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD) {
            cv::FileNode node = fSettings["Camera.bf"];
            if (!node.empty() && node.isReal()) {
                mbf = node.real();
                if (mImageScale != 1.f) {
                    mbf *= mImageScale;
                }
            } else {
                std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

        }

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;

        cout << "- fps: " << fps << endl;


        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD) {
            float fx = mpCamera->getParameter(0);
            cv::FileNode node = fSettings["ThDepth"];
            if (!node.empty() && node.isReal()) {
                mThDepth = node.real();
                mThDepth = mbf * mThDepth / fx;
                cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
            } else {
                std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


        }

        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
            cv::FileNode node = fSettings["DepthMapFactor"];
            if (!node.empty() && node.isReal()) {
                mDepthMapFactor = node.real();
                if (fabs(mDepthMapFactor) < 1e-5)
                    mDepthMapFactor = 1;
                else
                    mDepthMapFactor = 1.0f / mDepthMapFactor;
            } else {
                std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

        }

        if (b_miss_params) {
            return false;
        }

        return true;
    }

    bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings) {
        bool b_miss_params = false;
        int nFeatures, nLevels, fIniThFAST, fMinThFAST;
        float fScaleFactor;

        cv::FileNode node = fSettings["ORBextractor.nFeatures"];
        if (!node.empty() && node.isInt()) {
            nFeatures = node.operator int();
        } else {
            std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.scaleFactor"];
        if (!node.empty() && node.isReal()) {
            fScaleFactor = node.real();
        } else {
            std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.nLevels"];
        if (!node.empty() && node.isInt()) {
            nLevels = node.operator int();
        } else {
            std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.iniThFAST"];
        if (!node.empty() && node.isInt()) {
            fIniThFAST = node.operator int();
        } else {
            std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["ORBextractor.minThFAST"];
        if (!node.empty() && node.isInt()) {
            fMinThFAST = node.operator int();
        } else {
            std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        if (b_miss_params) {
            return false;
        }

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        return true;
    }

    bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings) {
        bool b_miss_params = false;

        cv::Mat cvTbc;
        cv::FileNode node = fSettings["Tbc"];
        if (!node.empty()) {
            cvTbc = node.mat();
            if (cvTbc.rows != 4 || cvTbc.cols != 4) {
                std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
                b_miss_params = true;
            }
        } else {
            std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
            b_miss_params = true;
        }
        cout << endl;
        cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
        Sophus::SE3f Tbc(eigTbc);

        node = fSettings["InsertKFsWhenLost"];
        mInsertKFsLost = true;
        if (!node.empty() && node.isInt()) {
            mInsertKFsLost = (bool) node.operator int();
        }

        if (!mInsertKFsLost)
            cout << "Do not insert keyframes when lost visual tracking " << endl;


        float Ng, Na, Ngw, Naw;

        node = fSettings["IMU.Frequency"];
        if (!node.empty() && node.isInt()) {
            mImuFreq = node.operator int();
            mImuPer = 0.001; //1.0 / (double) mImuFreq;
        } else {
            std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseGyro"];
        if (!node.empty() && node.isReal()) {
            Ng = node.real();
        } else {
            std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.NoiseAcc"];
        if (!node.empty() && node.isReal()) {
            Na = node.real();
        } else {
            std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.GyroWalk"];
        if (!node.empty() && node.isReal()) {
            Ngw = node.real();
        } else {
            std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.AccWalk"];
        if (!node.empty() && node.isReal()) {
            Naw = node.real();
        } else {
            std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["IMU.fastInit"];
        mFastInit = false;
        if (!node.empty()) {
            mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
        }

        if (mFastInit)
            cout << "Fast IMU initialization. Acceleration is not checked \n";

        if (b_miss_params) {
            return false;
        }

        const float sf = sqrt(mImuFreq);
        cout << endl;
        cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
        cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
        cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
        cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

        mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);


        return true;
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }

    void Tracking::SetStepByStep(bool bSet) {
        bStepByStep = bSet;
    }

    bool Tracking::GetStepByStep() {
        return bStepByStep;
    }


    Sophus::SE3f
    Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                              string filename) {
        //cout << "GrabImageStereo" << endl;

        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;
        mImRight = imRectRight;

        if (mImGray.channels() == 3) {
            //cout << "Image with 3 channels" << endl;
            if (mbRGB) {
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            //cout << "Image with 4 channels" << endl;
            if (mbRGB) {
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
            }
        }

        //cout << "Incoming frame creation" << endl;

        if (mSensor == System::STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                                  mpORBVocabulary,
                                  mK, mDistCoef, mbf, mThDepth, mpCamera);
        else if (mSensor == System::STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                                  mpORBVocabulary,
                                  mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr);
        else if (mSensor == System::IMU_STEREO && !mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                                  mpORBVocabulary,
                                  mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
        else if (mSensor == System::IMU_STEREO && mpCamera2)
            mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                                  mpORBVocabulary,
                                  mK, mDistCoef, mbf, mThDepth, mpCamera, mpCamera2, mTlr, &mLastFrame, *mpImuCalib);

        //cout << "Incoming frame ended" << endl;

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
        vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

        //cout << "Tracking start" << endl;
        Track();
        //cout << "Tracking end" << endl;

        return mCurrentFrame.GetPose();
    }


    Sophus::SE3f
    Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename) {

        cv::Mat imDepth = imD;

        mImRGB = imRGB;
        mImGray = imRGB;
        mImDepth = imD;

        if (mpSystem->isYoloDetection)
        {
            // Yolo
            cv::Mat InputImage;
            InputImage = imRGB.clone();
            mpDetector->GetImage(InputImage);
            mpDetector->Detect();
            mpORBextractorLeft->mvDynamicArea = mpDetector->mvDynamicArea;
            {
                std::unique_lock<std::mutex> lock(mpViewer->mMutexPAFinsh);
                mpViewer->mmDetectMap = mpDetector->mmDetectMap;
            }
        }

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || mImDepth.type() != CV_32F)
            mImDepth.convertTo(mImDepth, CV_32F, mDepthMapFactor);

        if (mSensor == System::RGBD)
            mCurrentFrame = Frame(mImGray, mImDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth, mpCamera);
        else if (mSensor == System::IMU_RGBD)
            mCurrentFrame = Frame(mImGray, mImDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth, mpCamera, &mLastFrame, *mpImuCalib);

        if(mpSystem->isYoloDetection)
        {
            mCurrentFrame.mvDynamicArea = mpDetector->mvDynamicArea;
            mpDetector->mmDetectMap.clear();
            mpDetector->mvDynamicArea.clear();
        }

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

        if (whether_detect_object)
            mCurrentFrame.raw_img = mImGray.clone();

        if (mCurrentFrame.mnId == 0)
        {
            mpMap->img_height = mImGray.rows;
            mpMap->img_width = mImGray.cols;
        }

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

        Track();

        return mCurrentFrame.GetPose();
    }


    Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename) {
        mImGray = im;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
        }

        if (mSensor == System::MONOCULAR) {

            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            {
                if( (!mono_firstframe_truth_depth_init) || (lastID - initID) < mMaxFrames) {
                    mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef,
                                          mbf,mThDepth);
                }
                else{
                    // read (truth) depth /stereo image for first frame
                    // not good to read first frame's predicted depth by object. quite inaccurate.
                    // orb slam will create left/right coordinates based on that, and will be used for
                    // optimizer.
                    std::string right_kitti_img_file = scene_folder + "/000000_right.png";
                    cv::Mat right_stereo_img = cv::imread(right_kitti_img_file, 0);
                    if (!right_stereo_img.data)
                        ROS_ERROR_STREAM("Cannot read first stereo file  " << right_kitti_img_file);
                    else
                        ROS_WARN_STREAM("Read first right stereo size  " << right_stereo_img.rows);
                    std::cout << "Read first right depth size  " << right_stereo_img.rows << "  baseline  "
                              << mbf << std::endl;
                    mCurrentFrame =
                            Frame(mImGray, right_stereo_img, timestamp, mpORBextractorLeft, mpORBextractorRight,
                                  mpORBVocabulary, mpCamera, mK, mDistCoef, mbf, mThDepth);
                }
            }
            else
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf,
                                      mThDepth);
        } else if (mSensor == System::IMU_MONOCULAR) {
            if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
                mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf,
                                      mThDepth, &mLastFrame, *mpImuCalib);
            } else
                mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf,
                                      mThDepth, &mLastFrame, *mpImuCalib);
        }

        if (mState == NO_IMAGES_YET)
            t0 = timestamp;

        mCurrentFrame.mNameFile = filename;
        mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
        vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

        lastID = mCurrentFrame.mnId;
        Track();

        return mCurrentFrame.GetPose();
    }


    void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
        unique_lock<mutex> lock(mMutexImuQueue);
        mlQueueImuData.push_back(imuMeasurement);
    }

    void Tracking::PreintegrateIMU() {

        if (!mCurrentFrame.mpPrevFrame) {
            Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated();
            return;
        }

        mvImuFromLastFrame.clear();
        mvImuFromLastFrame.reserve(mlQueueImuData.size());
        if (mlQueueImuData.size() == 0) {
            Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
            mCurrentFrame.setIntegrated();
            return;
        }

        while (true) {
            bool bSleep = false;
            {
                unique_lock<mutex> lock(mMutexImuQueue);
                if (!mlQueueImuData.empty()) {
                    IMU::Point *m = &mlQueueImuData.front();
                    cout.precision(17);
                    if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
                        mlQueueImuData.pop_front();
                    } else if (m->t < mCurrentFrame.mTimeStamp - mImuPer) {
                        mvImuFromLastFrame.push_back(*m);
                        mlQueueImuData.pop_front();
                    } else {
                        mvImuFromLastFrame.push_back(*m);
                        break;
                    }
                } else {
                    break;
                    bSleep = true;
                }
            }
            if (bSleep)
                usleep(500);
        }

        const int n = mvImuFromLastFrame.size() - 1;
        if (n == 0) {
            cout << "Empty IMU measurements vector!!!\n";
            return;
        }

        IMU::Preintegrated *pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,
                                                                                    mCurrentFrame.mImuCalib);

        for (int i = 0; i < n; i++) {
            float tstep;
            Eigen::Vector3f acc, angVel;
            if ((i == 0) && (i < (n - 1))) {
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
                float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                       (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                          (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) * 0.5f;
                tstep = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            } else if (i < (n - 1)) {
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
                tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            } else if ((i > 0) && (i == (n - 1))) {
                float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
                float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
                acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                       (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) * 0.5f;
                angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                          (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) * 0.5f;
                tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
            } else if ((i == 0) && (i == (n - 1))) {
                acc = mvImuFromLastFrame[i].a;
                angVel = mvImuFromLastFrame[i].w;
                tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
            }

            if (!mpImuPreintegratedFromLastKF)
                cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
            mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
            pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
        }

        mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

        mCurrentFrame.setIntegrated();

        //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
    }


    bool Tracking::PredictStateIMU() {
        if (!mCurrentFrame.mpPrevFrame) {
            Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        if (mbMapUpdated && mpLastKeyFrame) {
            const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            const float t12 = mpImuPreintegratedFromLastKF->dT;

            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
                    Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
            Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                                   Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
            Eigen::Vector3f Vwb2 =
                    Vwb1 + t12 * Gz +
                    Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
            return true;
        } else if (!mbMapUpdated) {
            const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
                    Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
            Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                                   Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
            Eigen::Vector3f Vwb2 =
                    Vwb1 + t12 * Gz +
                    Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

            mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            mCurrentFrame.mImuBias = mLastFrame.mImuBias;
            mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
            return true;
        } else
            cout << "not IMU prediction!!" << endl;

        return false;
    }

    void Tracking::ResetFrameIMU() {
        // TODO To implement...
    }


    void Tracking::Track() {

        if (bStepByStep) {
            std::cout << "Tracking: Waiting to the next step" << std::endl;
            while (!mbStep && bStepByStep)
                usleep(500);
            mbStep = false;
        }

        if (mpLocalMapper->mbBadImu) {
            cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
            mpSystem->ResetActiveMap();
            return;
        }

        Map *pCurrentMap = mpAtlas->GetCurrentMap();
        if (!pCurrentMap) {
            cout << "ERROR: There is not an active map in the atlas" << endl;
        }

        if (mState != NO_IMAGES_YET) {
            if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
                cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
                unique_lock<mutex> lock(mMutexImuQueue);
                mlQueueImuData.clear();
                CreateMapInAtlas();
                return;
            } else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0) {
                // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
                // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
                if (mpAtlas->isInertial()) {

                    if (mpAtlas->isImuInitialized()) {
                        cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                        if (!pCurrentMap->GetIniertialBA2()) {
                            mpSystem->ResetActiveMap();
                        } else {
                            CreateMapInAtlas();
                        }
                    } else {
                        cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                        mpSystem->ResetActiveMap();
                    }
                    return;
                }

            }
        }


        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            mpLastKeyFrame)
            mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            !mbCreatedMap) {
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
            PreintegrateIMU();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
vdIMUInteg_ms.push_back(timePreImu);
#endif

        }
        mbCreatedMap = false;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

        mbMapUpdated = false;

        int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
        int nMapChangeIndex = pCurrentMap->GetLastMapChange();
        if (nCurMapChangeIndex > nMapChangeIndex) {
            pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
            mbMapUpdated = true;
        }


        if (mState == NOT_INITIALIZED) {
            if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO ||
                mSensor == System::IMU_RGBD) {
                StereoInitialization();
            } else {
                bool special_initialization = false;
                if (mCurrentFrame.mnId == 0)
                {
                    if (mono_firstframe_truth_depth_init)
                    {
                        special_initialization = true;
                        StereoInitialization(); // if first frame has truth depth, we can initialize
                        // simiar to stereo/rgbd. create keyframe for it.
                    }
                    else if (mono_firstframe_Obj_depth_init)
                    {
                        special_initialization = true;
                        // similar to stereo initialization, but directly create map point. don't create
                        // stereo right coordinate have less effect on g2o optimization.  because depth
                        // initialization is not accurate
                        MonoObjDepthInitialization();
                    }
                }
                if (!special_initialization)
                    MonocularInitialization(); // usually for monocular, need to wait for several
                // frames, with enough parallax
            }

            //mpFrameDrawer->Update(this);

            if (mState != OK) // If rightly initialized, mState=OK
            {
                mLastFrame = Frame(mCurrentFrame);
                return;
            }

            if (mpAtlas->GetAllMaps().size() == 1) {
                mnFirstFrameId = mCurrentFrame.mnId;
            }
        } else {
            // System is initialized. Track Frame.
            bool bOK;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!mbOnlyTracking) {

                // State OK
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                if (mState == OK) {

                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
                        mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                        Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                        bOK = TrackReferenceKeyFrame();
                    } else {
                        Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }


                    if (!bOK) {
                        if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
                            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                             mSensor == System::IMU_RGBD)) {
                            mState = LOST;
                        } else if (pCurrentMap->KeyFramesInMap() > 10) {
                            // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                            mState = RECENTLY_LOST;
                            mTimeStampLost = mCurrentFrame.mTimeStamp;
                        } else {
                            mState = LOST;
                        }
                    }
                } else {

                    if (mState == RECENTLY_LOST) {
                        Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                        bOK = true;
                        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                             mSensor == System::IMU_RGBD)) {
                            if (pCurrentMap->isImuInitialized())
                                PredictStateIMU();
                            else
                                bOK = false;

                            if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost) {
                                mState = LOST;
                                Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                bOK = false;
                            }
                        } else {
                            // Relocalization
                            bOK = Relocalization();
                            //std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                            //std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                            if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
                                mState = LOST;
                                Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                                bOK = false;
                            }
                        }
                    } else if (mState == LOST) {

                        Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                        if (pCurrentMap->KeyFramesInMap() < 10) {
                            mpSystem->ResetActiveMap();
                            Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                        } else
                            CreateMapInAtlas();

                        if (mpLastKeyFrame)
                            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

                        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                        return;
                    }
                }

            } else {
                // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
                if (mState == LOST) {
                    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                        mSensor == System::IMU_RGBD)
                        Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                    bOK = Relocalization();
                } else {
                    if (!mbVO) {
                        // In last frame we tracked enough MapPoints in the map
                        if (mbVelocity) {
                            bOK = TrackWithMotionModel();
                        } else {
                            bOK = TrackReferenceKeyFrame();
                        }
                    } else {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        Sophus::SE3f TcwMM;
                        if (mbVelocity) {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.GetPose();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc) {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO) {
                                for (int i = 0; i < mCurrentFrame.N; i++) {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        } else if (bOKReloc) {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

            double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
            vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (!mbOnlyTracking) {
                if (bOK) {
                    bOK = TrackLocalMap();

                }
                if (!bOK)
                    cout << "Fail to track local map!" << endl;
            } else {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK)
                mState = OK;
            else if (mState == OK) {
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                    Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                    if (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2()) {
                        cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                        mpSystem->ResetActiveMap();
                    }

                    mState = RECENTLY_LOST;
                } else
                    mState = RECENTLY_LOST; // visual to lost

                /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
                    {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
                //}
            }

            // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
            if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
                (mCurrentFrame.mnId > mnFramesToResetIMU) &&
                (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
                pCurrentMap->isImuInitialized()) {
                // TODO check this situation
                Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
                Frame *pF = new Frame(mCurrentFrame);
                pF->mpPrevFrame = new Frame(mLastFrame);

                // Load preintegration
                pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            }

            if (pCurrentMap->isImuInitialized()) {
                if (bOK) {
                    if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
                        cout << "RESETING FRAME!!!" << endl;
                        ResetFrameIMU();
                    } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                        mLastBias = mCurrentFrame.mImuBias;
                }
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

            double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
            vdLMTrack_ms.push_back(timeLMTrack);
#endif

            // Update drawer
            mpFrameDrawer->Update(this);
            if (mCurrentFrame.isSet())
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            if (bOK || mState == RECENTLY_LOST) {
                // Update motion model
                if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
                    Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                    mVelocity = mCurrentFrame.GetPose() * LastTwc;
                    mbVelocity = true;
                } else {
                    mbVelocity = false;
                }

                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

                // Clean VO matches
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1) {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                     lit != lend; lit++) {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
                bool bNeedKF = NeedNewKeyFrame();

                // Check if we need to insert a new keyframe
                // if(bNeedKF && bOK)
                if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                                        (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                                         mSensor == System::IMU_RGBD))))
                    CreateNewKeyFrame();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

                double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
                vdNewKF_ms.push_back(timeNewKF);
#endif

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            if (mState == LOST) {
                if (pCurrentMap->KeyFramesInMap() <= 10) {
                    mpSystem->ResetActiveMap();
                    return;
                }
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    if (!pCurrentMap->isImuInitialized()) {
                        Verbose::PrintMess("Track lost before IMU initialisation, reseting...",
                                           Verbose::VERBOSITY_QUIET);
                        mpSystem->ResetActiveMap();
                        return;
                    }

                CreateMapInAtlas();

                return;
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }


        if (mState == OK || mState == RECENTLY_LOST) {
            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            if (mCurrentFrame.isSet()) {
                Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                mlRelativeFramePoses.push_back(Tcr_);
                mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState == LOST);
            } else {
                // This can happen if tracking is lost
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                mlFrameTimes.push_back(mlFrameTimes.back());
                mlbLost.push_back(mState == LOST);
            }

        }

#ifdef REGISTER_LOOP
        if (Stop()) {

        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
}
#endif
        if (whether_save_final_optimized_cuboids)
        {
            if ((mCurrentFrame.mnId >= final_object_record_frame_ind) && (!done_save_obj_to_txt))
            {
                SaveOptimizedCuboidsToTxt();
                done_save_obj_to_txt = true;
                ROS_WARN_STREAM("Done save cuboids to txt");
            }
        }
    }


    void Tracking::StereoInitialization() {
        if (mCurrentFrame.N > 500) {
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
                    cout << "not IMU meas" << endl;
                    return;
                }

                if (!mFastInit &&
                    (mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() <
                    0.5) {
                    cout << "not enough acceleration" << endl;
                    return;
                }

                if (mpImuPreintegratedFromLastKF)
                    delete mpImuPreintegratedFromLastKF;

                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            // Set Frame pose to the origin (In case of inertial SLAM to imu)
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
                Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
                Eigen::Vector3f Vwb0;
                Vwb0.setZero();
                mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
            } else
                mCurrentFrame.SetPose(Sophus::SE3f());

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpAtlas->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            if (!mpCamera2) {
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    float z = mCurrentFrame.mvDepth[i];
                    if (z > 0) {
                        Eigen::Vector3f x3D;
                        mCurrentFrame.UnprojectStereo(i, x3D);
                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                        pNewMP->AddObservation(pKFini, i);
                        pKFini->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    }
                }
            } else {
                for (int i = 0; i < mCurrentFrame.Nleft; i++) {
                    int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                    if (rightIndex != -1) {
                        Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                        MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                        pNewMP->AddObservation(pKFini, i);
                        pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

                        pKFini->AddMapPoint(pNewMP, i);
                        pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
                    }
                }
            }

            Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
                               Verbose::VERBOSITY_QUIET);

            //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;
            //mnLastRelocFrameId = mCurrentFrame.mnId;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

            mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            mState = OK;
        }
    }


    void Tracking::MonocularInitialization() {

        if (!mbReadyToInitializate) {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100) {

                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                if (mSensor == System::IMU_MONOCULAR) {
                    if (mpImuPreintegratedFromLastKF) {
                        delete mpImuPreintegratedFromLastKF;
                    }
                    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

                }

                mbReadyToInitializate = true;

                return;
            }
        } else {
            if (((int) mCurrentFrame.mvKeys.size() <= 100) ||
                ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
                mbReadyToInitializate = false;

                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                           100);

            // Check if there are enough correspondences
            if (nmatches < 100) {
                mbReadyToInitializate = false;
                return;
            }

            Sophus::SE3f Tcw;
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches, Tcw,
                                                  mvIniP3D,
                                                  vbTriangulated)) {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(Sophus::SE3f());
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::MonoObjDepthInitialization()
    {
        std::cout << "Come to Mono Object depth initialization !" << std::endl;
        ROS_WARN_STREAM("Created new keyframe!   " << 0 << "   total ID  " << 0);
        if (mCurrentFrame.N > 500)
        {
            // Set Frame pose to the origin
            if (build_worldframe_on_ground) // transform initial pose and map to ground frame
                mCurrentFrame.SetPose(GroundToInit);
            else
                mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame.    set (current) first frame as keyframe
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            if (whether_detect_object)
            {
                DetectCuboid(pKFini);
                AssociateCuboids(pKFini); // associate cuboids.  // first frame don't need
                // mpLastKeyFrame as I check .mID==0
            }

            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float point_depth = -1;
                cv::Mat x3D;
                if (whether_detect_object && mono_firstframe_Obj_depth_init &&
                    associate_point_with_object)
                {
                    int id = pKFini->keypoint_associate_objectID[i];
                    if (id > -1)
                    {
                        point_depth = pKFini->local_cuboids[id]->cube_meas.translation()[2]; // camera z
                        x3D = mCurrentFrame.UnprojectDepth(i, point_depth);
                    }
                }

                if (point_depth > 0)
                {
                    MapPoint *pNewMP =
                            new MapPoint(x3D, pKFini, mpMap); // x3d already world frame based on pose
                    pKFini->SetupSimpleMapPoints(pNewMP, i);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    mCurrentFrame.mvbOutlier[i] = false;
                }
            }

            cout << "New map created with " << mpMap->MapPointsInMap() << "  out of all  "
                 << mCurrentFrame.N << " feature points" << endl;

            if (mpMap->MapPointsInMap() == 0)
            {
                ROS_ERROR_STREAM("Bad MonoObjDepthInitialization! No map points! Break systems!");
                exit(0);
            }

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
        else
            cout << "Not enough points for initialization  " << mCurrentFrame.N << std::endl;
    }

    void Tracking::CreateInitialMapMonocular() {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        if (mSensor == System::IMU_MONOCULAR)
            pKFini->mpImuPreintegrated = (IMU::Preintegrated *) (NULL);

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpAtlas->AddKeyFrame(pKFini);
        mpAtlas->AddKeyFrame(pKFcur);

        if (whether_detect_object)
        {
            DetectCuboid(pKFini);
            AssociateCuboids(pKFini);
            DetectCuboid(pKFcur);
            AssociateCuboids(pKFcur);
        }

        for (size_t i = 0; i < mvIniMatches.size(); i++) {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            Eigen::Vector3f worldPos;

            if (mono_allframe_Obj_depth_init)
            {
                if (pKFini->KeysStatic.size() > 0 && !pKFini->KeysStatic[i])
                    continue;
                if (pKFcur->KeysStatic.size() > 0 && !pKFcur->KeysStatic[i])
                    continue;
            }

            worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpAtlas->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        std::set<MapPoint *> sMPs;
        sMPs = pKFini->GetMapPoints();

        // Bundle Adjustment
        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
                           Verbose::VERBOSITY_QUIET);
        Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth;
        if (mSensor == System::IMU_MONOCULAR)
            invMedianDepth = 4.0f / medianDepth; // 4.0f
        else
            invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
        {
            Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
            mpSystem->ResetActiveMap();
            return;
        }

        // Scale initial baseline
        Sophus::SE3f Tc2w = pKFcur->GetPose();
        Tc2w.translation() *= invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
            if (vpAllMapPoints[iMP]) {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
                pMP->UpdateNormalAndDepth();
            }
        }

        if (build_worldframe_on_ground) // transform initial pose and map to ground frame
        {
            pKFini->SetPose(pKFini->GetPose() * GroundToInit);
            pKFcur->SetPose(pKFcur->GetPose() * GroundToInit);

            for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
            {
                if (vpAllMapPoints[iMP])
                {
                    MapPoint *pMP = vpAllMapPoints[iMP];
                    pMP->SetWorldPos(InitToGround.rowRange(0, 3).colRange(0, 3) * pMP->GetWorldPos() +
                                     InitToGround.rowRange(0, 3).col(3));
                }
            }
        }

        if (mSensor == System::IMU_MONOCULAR) {
            pKFcur->mPrevKF = pKFini;
            pKFini->mNextKF = pKFcur;
            pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),
                                                                  pKFcur->mImuCalib);
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);
        mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;
        //mnLastRelocFrameId = mInitialFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        // Compute here initial velocity
        vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

        Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
        mbVelocity = false;
        Eigen::Vector3f phi = deltaT.so3().log();

        double aux =
                (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) /
                (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
        phi *= aux;

        mLastFrame = Frame(mCurrentFrame);

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;

        initID = pKFcur->mnId;
    }


    void Tracking::CreateMapInAtlas() {
        mnLastInitFrameId = mCurrentFrame.mnId;
        mpAtlas->CreateNewMap();
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
            mpAtlas->SetInertialSensor();
        mbSetInit = false;

        mnInitialFrameId = mCurrentFrame.mnId + 1;
        mState = NO_IMAGES_YET;

        // Restart the variable with information about the last KF
        mbVelocity = false;
        //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
        Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL);
        mbVO = false; // Init value for know if there are enough MapPoints in the last KF
        if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
            mbReadyToInitializate = false;
        }

        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            mpImuPreintegratedFromLastKF) {
            delete mpImuPreintegratedFromLastKF;
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        }

        if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

        if (mpReferenceKF)
            mpReferenceKF = static_cast<KeyFrame *>(NULL);

        mLastFrame = Frame();
        mCurrentFrame = Frame();
        mvIniMatches.clear();

        mbCreatedMap = true;
    }

    void Tracking::CheckReplacedInLastFrame() {
        for (int i = 0; i < mLastFrame.N; i++) {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP) {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep) {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }


    bool Tracking::TrackReferenceKeyFrame() {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15) {
            cout << "TRACK_REF_KF: Less than 15 matches!!\n";
            return false;
        }

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.GetPose());

        //mCurrentFrame.PrintPointDistribution();


        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            //if(i >= mCurrentFrame.Nleft) break;
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    if (i < mCurrentFrame.Nleft) {
                        pMP->mbTrackInView = false;
                    } else {
                        pMP->mbTrackInViewR = false;
                    }
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    if (!(whether_dynamic_object && mCurrentFrame.mvpMapPoints[i]->is_dynamic))
                        nmatchesMap++;
            }
        }

        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame() {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        Sophus::SE3f Tlr = mlRelativeFramePoses.back();
        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR ||
            !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int> > vDepthIdx;
        const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
        vDepthIdx.reserve(Nfeat);
        for (int i = 0; i < Nfeat; i++) {
            float z = mLastFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++) {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
                bCreateNew = true;

            if (bCreateNew) {
                Eigen::Vector3f x3D;

                if (mLastFrame.Nleft == -1) {
                    mLastFrame.UnprojectStereo(i, x3D);
                } else {
                    x3D = mLastFrame.UnprojectStereoFishEye(i);
                }

                MapPoint *pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            } else {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;

        }
    }

    bool Tracking::TrackWithMotionModel() {
        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
            // Predict state with IMU if it is initialized and it doesnt need reset
            PredictStateIMU();
            return true;
        } else {
            mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
        }


        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;

        if (mSensor == System::STEREO)
            th = 7;
        else
            th = 15;

        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th,
                                                  mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

        // If few matches, uses a wider window search
        if (nmatches < 20) {
            Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th,
                                                  mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
            Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

        }

        // Project map points seen in previous frame  onto current frame.
        int searchRadiusFactor;
        if (mSensor != System::STEREO)
            searchRadiusFactor = 15;
        else
            searchRadiusFactor = 7;

        if (whether_detect_object)
        {
            if (use_dynamic_klt_features)
                matcher.SearchByTrackingHarris(mCurrentFrame, mLastFrame, searchRadiusFactor,
                                               mSensor == System::MONOCULAR);
            else
                matcher.SearchByTracking(mCurrentFrame, mLastFrame, searchRadiusFactor,
                                         mSensor == System::MONOCULAR);
        }

        if (nmatches < 20) {
            Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                return true;
            else
                return false;
        }

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    if (i < mCurrentFrame.Nleft) {
                        pMP->mbTrackInView = false;
                    } else {
                        pMP->mbTrackInViewR = false;
                    }
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    if (!(whether_dynamic_object && mCurrentFrame.mvpMapPoints[i]->is_dynamic))
                        nmatchesMap++;
            }
        }

        if (mbOnlyTracking) {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap() {

        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        mTrackedFr++;

        UpdateLocalMap();
        SearchLocalPoints();

        // TOO check outliers before PO
        int aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i]) {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        int inliers;
        if (!mpAtlas->isImuInitialized())
            Optimizer::PoseOptimization(&mCurrentFrame);
        else {
            if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
                Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                Optimizer::PoseOptimization(&mCurrentFrame);
            } else {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                if (!mbMapUpdated) //  && (mnMatchesInliers>30))
                {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastFrame(
                            &mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                } else {
                    Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(
                            &mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                }
            }
        }

        aux1 = 0, aux2 = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
            if (mCurrentFrame.mvpMapPoints[i]) {
                aux1++;
                if (mCurrentFrame.mvbOutlier[i])
                    aux2++;
            }

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    if (whether_dynamic_object && mCurrentFrame.mvpMapPoints[i]->is_dynamic)
                        continue;
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();

                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        int map_match_thres = 30;  // NOTE important raw: 30
        if (whether_detect_object) // for dynamic scenarios, there might not be enough feature matches
            map_match_thres = 20;

        if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
            return true;

        if (mSensor == System::IMU_MONOCULAR) {
            if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
                (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized())) {
                return false;
            } else
                return true;
        } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            if (mnMatchesInliers < 15) {
                return false;
            } else
                return true;
        } else {
            if (mnMatchesInliers < 30)
                return false;
            else
                return true;
        }
    }

    bool Tracking::NeedNewKeyFrame() {
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            !mpAtlas->GetCurrentMap()->isImuInitialized()) {
            if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
                     (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
                return true;
            else
                return false;
        }

        if (mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
            /*if(mSensor == System::MONOCULAR)
              {
                  std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
              }*/
            return false;
        }

        const int nKFs = mpAtlas->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames) {
            return false;
        }

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;

        // if first frame depth initialized by object... not actual measurement. map point only have one
        // observation need to reduce the threshold for initialization.
        if ((mono_firstframe_Obj_depth_init || mono_firstframe_truth_depth_init))
        {
            if ((mSensor == System::MONOCULAR) && (scene_unique_id == kitti) &&
                (mpReferenceKF->mnId < 20)) // for kitti, don't need that many
                nMinObs = 1;
        }

        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;

        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
            int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
            for (int i = 0; i < N; i++) {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;

                }
            }
            //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
        }

        bool bNeedToInsertClose;
        bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
        const int thStereoClosedPoints = 15;
        if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
        {
            //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
            thRefRatio = 0.9f;
        }*/

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        if (mpCamera2) thRefRatio = 0.75f;

        if (mSensor == System::IMU_MONOCULAR) {
            if (mnMatchesInliers > 350) // Points tracked from the local map
                thRefRatio = 0.75f;
            else
                thRefRatio = 0.90f;
        }

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
                          bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
        //Condition 1c: tracking is weak
        const bool c1c =
                mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR && mSensor != System::IMU_STEREO &&
                mSensor != System::IMU_RGBD && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
                         mnMatchesInliers > 15);

        //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
        // Temporal condition for Inertial cases
        bool c3 = false;
        if (mpLastKeyFrame) {
            if (mSensor == System::IMU_MONOCULAR) {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
                if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                    c3 = true;
            }
        }

        bool c4 = false;
        if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == RECENTLY_LOST) && (mSensor ==
                                                                                                  System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
            c4 = true;
        else
            c4 = false;

        if (((c1a || c1b || c1c) && c2) || c3 || c4) {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
                return true;
            } else {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                } else {
                    //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                    return false;
                }
            }
        } else
            return false;
    }

    void Tracking::AssociateCuboids(KeyFrame *pKF)
    {
        // loop over current KF's objects, check with all past objects (or local objects), compare the
        // associated object map points. (if a local object is not associated, could re-check here as
        // frame-object-point might change overtime, especially due to triangulation.)

        std::vector<MapObject *> LocalObjectsCandidates;
        std::vector<MapObject *> LocalObjectsLandmarks;
        // keypoint might not added to frame observation yet, so object might not have many associated
        // points yet.... method 1: just check current frame's object, using existing map point
        // associated objects. same as plane association, don't just check current frame, but check all
        // recent keyframe's unmatched objects...
        for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++) // pKF is not in mvpLocalKeyFrames yet
        {
            KeyFrame *kfs = mvpLocalKeyFrames[i];
            for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
            {
                MapObject *mPO = kfs->local_cuboids[j];
                if (mPO->become_candidate && (!mPO->already_associated))
                    LocalObjectsCandidates.push_back(kfs->local_cuboids[j]);
            }
            for (size_t j = 0; j < kfs->cuboids_landmark.size(); j++)
                if (kfs->cuboids_landmark[j]) // might be deleted due to badFlag()
                    if (!kfs->cuboids_landmark[j]->isBad())
                        if (kfs->cuboids_landmark[j]->association_refid_in_tracking !=
                            pKF->mnId) // could also use set to avoid duplicates
                        {
                            LocalObjectsLandmarks.push_back(kfs->cuboids_landmark[j]);
                            kfs->cuboids_landmark[j]->association_refid_in_tracking = pKF->mnId;
                        }
        }

        std::cout << "begin to associate cuboids #candidate:   " << LocalObjectsCandidates.size()
                  << "   #landmarks   " << LocalObjectsLandmarks.size() << "   #localKFs   "
                  << mvpLocalKeyFrames.size() << std::endl;
        int largest_shared_num_points_thres = 10;
        if (mono_allframe_Obj_depth_init)
            largest_shared_num_points_thres = 20;
        if (scene_unique_id == kitti)
            largest_shared_num_points_thres = 10; // kitti vehicle occupy large region

        if (whether_detect_object &&
            mono_allframe_Obj_depth_init) // dynamic object is more difficult. especially reverse motion
            largest_shared_num_points_thres = 5;

        MapObject *last_new_created_object = nullptr;
        for (size_t i = 0; i < LocalObjectsCandidates.size(); i++)
        {
            // there might be some new created object!
            if (last_new_created_object)
                LocalObjectsLandmarks.push_back(last_new_created_object);
            last_new_created_object = nullptr;

            // find existing object landmarks which share most points with this object
            MapObject *candidateObject = LocalObjectsCandidates[i];
            std::vector<MapPoint *> object_owned_pts = candidateObject->GetPotentialMapPoints();

            MapObject *largest_shared_objectlandmark = nullptr;
            if (LocalObjectsLandmarks.size() > 0)
            {
                map<MapObject *, int> LandmarkObserveCounter;

                for (size_t j = 0; j < object_owned_pts.size(); j++)
                    for (map<MapObject *, int>::iterator mit =
                            object_owned_pts[j]->MapObjObservations.begin();
                         mit != object_owned_pts[j]->MapObjObservations.end(); mit++)
                        LandmarkObserveCounter[mit->first]++;

                int largest_shared_num_points = largest_shared_num_points_thres;
                for (size_t j = 0; j < LocalObjectsLandmarks.size(); j++)
                {
                    MapObject *pMP = LocalObjectsLandmarks[j];
                    if (!pMP->isBad())
                        if (LandmarkObserveCounter.count(pMP))
                        {
                            if (LandmarkObserveCounter[pMP] > largest_shared_num_points)
                            {
                                largest_shared_num_points = LandmarkObserveCounter[pMP];
                                largest_shared_objectlandmark = pMP;
                            }
                        }
                }
            }

            if (use_truth_trackid) // find associate id based on tracket id.
            {
                if (trackletid_to_landmark.count(candidateObject->truth_tracklet_id))
                    largest_shared_objectlandmark =
                            trackletid_to_landmark[candidateObject->truth_tracklet_id];
                else
                    largest_shared_objectlandmark == nullptr;
            }

            if (largest_shared_objectlandmark ==
                nullptr) // if not found, create as new landmark.  either using original local pointer,
                // or initialize as new
            {
                if (use_truth_trackid)
                {
                    if (candidateObject->truth_tracklet_id >
                        -1) // -1 means no ground truth tracking ID, don't use this object
                        trackletid_to_landmark[candidateObject->truth_tracklet_id] = candidateObject;
                    else
                        continue;
                }
                candidateObject->already_associated = true; // must be put before SetAsLandmark();
                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
                candidateObject->addObservation(
                        refframe, candidateObject->object_id_in_localKF); // add to frame observation
                refframe->cuboids_landmark.push_back(candidateObject);
                candidateObject->mnId = MapObject::getIncrementedIndex(); // mpMap->MapObjectsInMap();
                // // needs to manually set
                candidateObject->associated_landmark = candidateObject;
                candidateObject->SetAsLandmark();
                if (scene_unique_id == kitti) // object scale change back and forth
                {
                    g2o::cuboid cubeglobalpose = candidateObject->GetWorldPos();
                    cubeglobalpose.setScale(Eigen::Vector3d(1.9420, 0.8143, 0.7631));
                    candidateObject->SetWorldPos(cubeglobalpose);
                    candidateObject->pose_Twc_latestKF = cubeglobalpose;
                    candidateObject->pose_noopti = cubeglobalpose;
                    candidateObject->allDynamicPoses[refframe] =
                            make_pair(cubeglobalpose, false); // Vector6d::Zero()  false means not BAed
                }
                mpMap->AddMapObject(candidateObject);
                last_new_created_object = candidateObject;
                candidateObject->allDynamicPoses[refframe] =
                        make_pair(candidateObject->GetWorldPos(), false);
            }
            else // if found, then update observation.
            {
                candidateObject->already_associated = true; // must be put before SetAsLandmark();
                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
                largest_shared_objectlandmark->addObservation(refframe,
                                                              candidateObject->object_id_in_localKF);
                refframe->cuboids_landmark.push_back(largest_shared_objectlandmark);
                candidateObject->associated_landmark = largest_shared_objectlandmark;

                // NOTE use current frame's object poes, but don't use current object if very close to
                // boundary.... large error
                // I use this mainly for kitti, as further objects are inaccurate.  for indoor object,
                // we may not need it
                if (scene_unique_id == kitti)
                {
                    g2o::cuboid cubeglobalpose = candidateObject->GetWorldPos();
                    cubeglobalpose.setScale(Eigen::Vector3d(1.9420, 0.8143, 0.7631));

                    largest_shared_objectlandmark->allDynamicPoses[refframe] =
                            make_pair(cubeglobalpose, false);
                    largest_shared_objectlandmark->SetWorldPos(cubeglobalpose);
                    largest_shared_objectlandmark->pose_Twc_latestKF =
                            cubeglobalpose; // if want to test without BA
                    largest_shared_objectlandmark->pose_noopti = cubeglobalpose;
                }
                largest_shared_objectlandmark->MergeIntoLandmark(candidateObject);
            }
        }

        // remove outlier objects....
        bool remove_object_outlier = true;

        int minimum_object_observation = 2;
        if (scene_unique_id == kitti)
        {
            remove_object_outlier = false;
            if (whether_detect_object)
            {
                remove_object_outlier = true;
                minimum_object_observation = 3; // dynamic object has more outliers
            }
        }

        bool check_object_points = true;

        if (remove_object_outlier)
        {
            vector<MapObject *> all_objects = mpMap->GetAllMapObjects();
            for (size_t i = 0; i < all_objects.size(); i++)
            {
                MapObject *pMObject = all_objects[i];
                if ((!pMObject->isBad()) && (!pMObject->isGood)) // if not determined good or bad yet.
                    if ((int)pMObject->GetLatestKeyFrame()->mnId < (int)pKF->mnId - 15) // 20
                    {
                        // if not recently observed, and not enough observations.  NOTE if point-object
                        // not used in BA, filtered size will be zero...
                        bool no_enough_inlier_pts =
                                check_object_points && (pMObject->NumUniqueMapPoints() > 20) &&
                                (pMObject->used_points_in_BA_filtered.size() < 10) &&
                                (pMObject->point_object_BA_counter > -1);
                        if (pMObject->Observations() < minimum_object_observation)
                        {
                            pMObject->SetBadFlag();
                            cout << "Found one bad object !!!!!!!!!!!!!!!!!!!!!!!!!  " << pMObject->mnId
                                 << "  " << pMObject->Observations() << "  "
                                 << pMObject->used_points_in_BA_filtered.size() << endl;

                            if (use_truth_trackid)
                                trackletid_to_landmark.erase(
                                        pMObject->truth_tracklet_id); // remove from track id mapping
                        }
                        else
                        {
                            pMObject->isGood = true;
                        }
                    }
            }
        }
    }

    template <class BidiIter> // Fisher-Yates shuffle
    BidiIter random_unique2(BidiIter begin, BidiIter end, int num_random)
    {
        size_t left = std::distance(begin, end);
        while (num_random--)
        {
            BidiIter r = begin;
            std::advance(r, rand() % left);
            std::swap(*begin, *r);
            ++begin;
            --left;
        }
        return begin;
    }

    void Tracking::CreateNewKeyFrame() {
        if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
            return;

        if (!mpLocalMapper->SetNotStop(true))
            return;

        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        if (mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
            pKF->bImu = true;

        pKF->SetNewBias(mCurrentFrame.mImuBias);
        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (whether_detect_object)
        {
            DetectCuboid(pKF);
            AssociateCuboids(pKF);
        }

        if (mpLastKeyFrame) {
            pKF->mPrevKF = mpLastKeyFrame;
            mpLastKeyFrame->mNextKF = pKF;
        } else
            Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

        // Reset preintegration from last KF (Create new object)
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
        }

        if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
        {
            mCurrentFrame.UpdatePoseMatrices();
            // cout << "create new MPs" << endl;
            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            int maxPoint = 100;
            if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                maxPoint = 100;

            vector<pair<float, int> > vDepthIdx;
            int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty()) {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++) {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1) {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew) {
                        Eigen::Vector3f x3D;

                        if (mCurrentFrame.Nleft == -1) {
                            mCurrentFrame.UnprojectStereo(i, x3D);
                        } else {
                            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                        }

                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                        pNewMP->AddObservation(pKF, i);

                        //Check if it is a stereo observation in order to not
                        //duplicate mappoints
                        if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
                            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft +
                                                       mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                            pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        }

                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpAtlas->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    } else {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
                        break;
                    }
                }
                //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
            }
        }

        // copied from localMapping, only for dynamic object
        if (mono_allframe_Obj_depth_init && whether_dynamic_object) {
            KeyFrame *mpCurrentKeyFrame = pKF;

            const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

            double total_feature_pts = vpMapPointMatches.size();
            double raw_depth_pts = 0; // point already have depth
            double plane_object_initialized_pts = 0;
            std::vector<int> raw_pixels_no_depth_inds;

            if (triangulate_dynamic_pts) {
                vector<MapPoint *> frameMapPointMatches;
                if (use_dynamic_klt_features)
                    frameMapPointMatches = mCurrentFrame.mvpMapPointsHarris;
                else
                    frameMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

                cv::Mat Tcw_last = mpLastKeyFrame->GetPose();
                cv::Mat Tcw_now = mpCurrentKeyFrame->GetPose();
                const float &cx1 = mpCurrentKeyFrame->cx;
                const float &cy1 = mpCurrentKeyFrame->cy;
                const float &invfx1 = mpCurrentKeyFrame->invfx;
                const float &invfy1 = mpCurrentKeyFrame->invfy;
                for (size_t i = 0; i < frameMapPointMatches.size(); i++) {
                    MapPoint *pMP = frameMapPointMatches[i];
                    if (pMP && pMP->is_dynamic) // the point is matched to this frame, and also dynamic.
                    {
                        // check if this point is created by last keyframe, if yes, triangulate it with
                        // this frame!   if created earlier, not need
                        if (!pMP->is_triangulated) // if not Triangulated
                        {
                            int pixelindLastKf = pMP->GetIndexInKeyFrame(mpLastKeyFrame);
                            if (pixelindLastKf == -1) {
                                ROS_ERROR_STREAM("Point frame observation not added yet");
                                continue;
                            }
                            MapObject *objectLastframe;
                            if (use_dynamic_klt_features)
                                objectLastframe =
                                        mpLastKeyFrame->local_cuboids
                                        [mpLastKeyFrame
                                                ->keypoint_associate_objectID_harris[pixelindLastKf]];
                            else
                                objectLastframe =
                                        mpLastKeyFrame->local_cuboids
                                        [mpLastKeyFrame->keypoint_associate_objectID[pixelindLastKf]];
                            g2o::cuboid cube_pose_lastkf;
                            if (objectLastframe->already_associated)
                                cube_pose_lastkf = objectLastframe->associated_landmark
                                        ->allDynamicPoses[mpLastKeyFrame]
                                        .first;
                            else
                                cube_pose_lastkf = objectLastframe->GetWorldPos();
                            // get new cube pose in this frame??? based on keypoint object asscoiate id.
                            MapObject *objectThisframe;
                            if (use_dynamic_klt_features)
                                objectThisframe =
                                        mpCurrentKeyFrame->local_cuboids
                                        [mCurrentFrame.keypoint_associate_objectID_harris[i]];
                            else
                                objectThisframe =
                                        mpCurrentKeyFrame->local_cuboids
                                        [mpCurrentKeyFrame->keypoint_associate_objectID[i]];
                            g2o::cuboid cube_pose_now =
                                    objectThisframe->GetWorldPos(); // current obj pose, not BA optimimized
                            // check truth tracklet id.
                            if (use_truth_trackid)
                                if (objectLastframe->truth_tracklet_id !=
                                    objectThisframe->truth_tracklet_id) {
                                    ROS_ERROR_STREAM("Different object tracklet id, possibly due to "
                                                     "wrong KLT point tracking");
                                    continue;
                                }
                            g2o::SE3Quat objecttransform =
                                    cube_pose_now.pose * cube_pose_lastkf.pose.inverse();
                            cv::Mat Tcw_now_withdynamic = Tcw_now * Converter::toCvMat(objecttransform);

                            cv::KeyPoint kp1, kp2;
                            if (use_dynamic_klt_features) {
                                kp1 = mpLastKeyFrame->mvKeysHarris[pixelindLastKf];
                                kp2 = mCurrentFrame.mvKeysHarris[i];
                            } else {
                                kp1 = mpLastKeyFrame->mvKeysUn[pixelindLastKf];
                                kp2 = mpCurrentKeyFrame->mvKeysUn[i];
                            }
                            // Check parallax between rays
                            cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1,
                                    (kp1.pt.y - cy1) * invfy1, 1.0);
                            cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx1) * invfx1,
                                    (kp2.pt.y - cy1) * invfy1, 1.0);

                            cv::Mat x3D;
                            {
                                // Linear Triangulation Method
                                cv::Mat A(4, 4, CV_32F);
                                A.row(0) = xn1.at<float>(0) * Tcw_last.row(2) - Tcw_last.row(0);
                                A.row(1) = xn1.at<float>(1) * Tcw_last.row(2) - Tcw_last.row(1);
                                A.row(2) = xn2.at<float>(0) * Tcw_now_withdynamic.row(2) -
                                           Tcw_now_withdynamic.row(0);
                                A.row(3) = xn2.at<float>(1) * Tcw_now_withdynamic.row(2) -
                                           Tcw_now_withdynamic.row(1);

                                cv::Mat w, u, vt;
                                cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

                                x3D = vt.row(3).t();

                                if (x3D.at<float>(3) == 0)
                                    continue;

                                // Euclidean coordinates
                                x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
                            }

                            if ((Converter::toVector3f(x3D) - pMP->GetWorldPosVec()).norm() >
                                5) // if too far from object center, not good triangulation.
                            {
                                continue;
                            }
                            pMP->is_triangulated = true;
                            pMP->PosToObj = Converter::toCvMat(
                                    cube_pose_now.pose.inverse().map(Converter::toVector3d(x3D)));
                        }
                    }
                }
            }


            if (1) // randomly select N points, don't initialize all of them
            {
                bool actually_use_obj_depth = false;
                if (mono_allframe_Obj_depth_init && whether_detect_object &&
                    associate_point_with_object)
                    if (mpCurrentKeyFrame->keypoint_associate_objectID.size() > 0)
                        actually_use_obj_depth = true;

                if (actually_use_obj_depth) {
                    cout << "Tracking just about to initialize object depth point" << endl;

                    // detect new KLF feature points   away from existing featute points.
                    if (use_dynamic_klt_features) {
                        // loop over all mappoints, create circle mask
                        // 		    objmask_img:  0 background, >0 object areas   change to--->   255
                        // object areas. 0 background.
                        int MIN_DIST = 10; // or 10  20
                        cv::Mat good_mask; // area to generate new features.
                        threshold(mCurrentFrame.objmask_img, good_mask, 0, 255,
                                  cv::THRESH_BINARY); // threshold to be 0,255

                        // final existing object mappoints.
                        vector<pair<int, cv::Point2f>> exist_obj_mappts; // point, obverse time

                        for (size_t i = 0; i < mCurrentFrame.mvpMapPointsHarris.size(); i++) {
                            MapPoint *pMP =
                                    mCurrentFrame.mvpMapPointsHarris[i]; // TODO later should be separate
                            // mappoint for dynamic
                            if (pMP && pMP->is_dynamic) {
                                exist_obj_mappts.push_back(
                                        make_pair(pMP->Observations(), mCurrentFrame.mvKeysHarris[i].pt));
                            }
                        }
                        // sort(exist_obj_mappts.begin(), exist_obj_mappts.end(), [](MapPoint *a,
                        // MapPoint *b) { return a->Observations() > b->Observations(); });
                        sort(exist_obj_mappts.begin(), exist_obj_mappts.end(),
                             [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b) {
                                 return a.first > b.first;
                             });

                        for (auto &it: exist_obj_mappts) {
                            if (good_mask.at<uchar>(it.second) == 255) {
                                cv::circle(good_mask, it.second, MIN_DIST, 0, -1);
                            }
                        }
                        cout << "mCurrentFrame.mvpMapPointsHarris size   "
                             << mCurrentFrame.mvpMapPointsHarris.size() << "  "
                             << exist_obj_mappts.size() << endl;

                        int max_new_pts = 200; // 100
                        vector<cv::Point2f> corners;
                        cv::goodFeaturesToTrack(mpCurrentKeyFrame->raw_img, corners, max_new_pts, 0.1,
                                                MIN_DIST, good_mask);

                        int numTracked = mCurrentFrame.mvKeysHarris.size();
                        int numNewfeat = corners.size();
                        int totalfeat = numTracked + numNewfeat;

                        mpCurrentKeyFrame->mvKeysHarris = mCurrentFrame.mvKeysHarris;
                        mpCurrentKeyFrame->mvKeysHarris.resize(totalfeat);
                        mpCurrentKeyFrame->mvpMapPointsHarris = mCurrentFrame.mvpMapPointsHarris;
                        mpCurrentKeyFrame->mvpMapPointsHarris.resize(totalfeat);
                        mpCurrentKeyFrame->keypoint_associate_objectID_harris =
                                mCurrentFrame.keypoint_associate_objectID_harris;
                        mpCurrentKeyFrame->keypoint_associate_objectID_harris.resize(totalfeat);

                        // create and append new detected features.
                        for (int new_fea_ind = 0; new_fea_ind < numNewfeat; new_fea_ind++) {
                            int maskval = int(mCurrentFrame.objmask_img.at<uchar>(
                                    corners[new_fea_ind])); // 0 background, >0 object id
                            int pixelcubeid = maskval - 1;
                            if (maskval == 0) {
                                ROS_ERROR_STREAM("Get invalid pixel object index");
                                exit(0);
                            }
                            cv::KeyPoint keypt;
                            keypt.pt = corners[new_fea_ind];
                            mpCurrentKeyFrame->mvKeysHarris[new_fea_ind + numTracked] = keypt;
                            mpCurrentKeyFrame
                                    ->keypoint_associate_objectID_harris[new_fea_ind + numTracked] =
                                    pixelcubeid;

                            float point_depth = mpCurrentKeyFrame->local_cuboids[pixelcubeid]
                                    ->cube_meas.translation()[2]; // camera z
                            cv::Mat x3D = mpCurrentKeyFrame->UnprojectPixelDepth(corners[new_fea_ind],
                                                                                 point_depth);

                            MapPoint *pNewMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);
                            pNewMP->is_dynamic = true;
                            mpCurrentKeyFrame->SetupSimpleMapPoints(
                                    pNewMP,
                                    new_fea_ind + numTracked); // add to frame observation, add to map.
                            // also changed mvpMapPointsHarris
                            pNewMP->is_triangulated = false;
                            pNewMP->SetWorldPos(x3D); // compute dynamic point to object pose
                        }
                        cout << "tracked/new_created features   " << numTracked << "  " << numNewfeat
                             << endl;

                        // update total features in mCurrentFrame
                        mCurrentFrame.mvKeysHarris = mpCurrentKeyFrame->mvKeysHarris;
                        mCurrentFrame.mvpMapPointsHarris = mpCurrentKeyFrame->mvpMapPointsHarris;
                        mCurrentFrame.keypoint_associate_objectID_harris =
                                mpCurrentKeyFrame->keypoint_associate_objectID_harris;
                    } else {
                        std::vector<int> has_object_depth_pixel_inds; // points with no depth yet but
                        // with matching object and plane

                        bool gridsHasMappt[FRAME_GRID_COLS][FRAME_GRID_ROWS];
                        for (int i = 0; i < FRAME_GRID_COLS; i++)
                            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                                gridsHasMappt[i][j] = false;
                        for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
                            MapPoint *pMP = vpMapPointMatches[i];
                            if (!pMP) // no map point yet. not associated yet
                            {
                                int gridx, gridy;
                                if (mpCurrentKeyFrame->PosInGrid(mpCurrentKeyFrame->mvKeys[i], gridx,
                                                                 gridy)) {
                                    if (gridsHasMappt[gridx][gridy])
                                        continue;
                                } else
                                    continue;

                                if (mpCurrentKeyFrame->keypoint_associate_objectID[i] >
                                    -1) // have associated object
                                    if (mpCurrentKeyFrame->mvKeys[i].octave <
                                        3) // HACK for KLT tracking, better just use first octave
                                    {
                                        has_object_depth_pixel_inds.push_back(i);
                                        gridsHasMappt[gridx][gridy] = true;
                                    }
                            } else
                                raw_depth_pts++;
                        }
                        bool whether_actually_planeobj_init_pt = false;

                        double depth_point_ration_now = raw_depth_pts / total_feature_pts;
                        int max_initialize_pts = 0;
                        if (depth_point_ration_now < 0.30) // 0.3
                            whether_actually_planeobj_init_pt = true;
                        max_initialize_pts =
                                std::min(int(total_feature_pts * 0.30) - int(raw_depth_pts),
                                         int(has_object_depth_pixel_inds.size()));
                        max_initialize_pts = std::min(max_initialize_pts, 80);

                        cout << "all points to initilaze  " << has_object_depth_pixel_inds.size()
                             << "  initialized " << max_initialize_pts << endl;
                        int nPoints = 0;

                        if (whether_actually_planeobj_init_pt) {
                            srand(time(NULL));
                            // 		    random_shuffle ( has_object_depth_pixel_inds.begin(),
                            // has_object_depth_pixel_inds.end() );
                            random_unique2(has_object_depth_pixel_inds.begin(),
                                           has_object_depth_pixel_inds.end(), max_initialize_pts);

                            int vector_counter = 0;
                            while ((nPoints < max_initialize_pts) &&
                                   (vector_counter < (int) has_object_depth_pixel_inds.size())) {
                                int pixel_ind = has_object_depth_pixel_inds[vector_counter];
                                float point_depth = -1;
                                cv::Mat x3D;

                                if ((point_depth < 0)) {
                                    if (mpCurrentKeyFrame->keypoint_associate_objectID[pixel_ind] > -1) {
                                        point_depth =
                                                mpCurrentKeyFrame
                                                        ->local_cuboids
                                                [mpCurrentKeyFrame
                                                        ->keypoint_associate_objectID[pixel_ind]]
                                                        ->cube_meas.translation()[2]; // camera z
                                        x3D = mpCurrentKeyFrame->UnprojectDepth(pixel_ind, point_depth);
                                    }
                                }
                                if (point_depth > 0) {
                                    MapPoint *pNewMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);
                                    mpCurrentKeyFrame->SetupSimpleMapPoints(
                                            pNewMP, pixel_ind); // add to frame observation, add to map.
                                    pNewMP->is_triangulated = false;
                                    nPoints++;
                                    if (whether_dynamic_object) {
                                        pNewMP->is_dynamic = true;
                                    }
                                } else {
                                    // NOTE projected point is negative. remove association? because
                                    // this point is bad
                                }
                                vector_counter++;
                            }
                            plane_object_initialized_pts = nPoints;
                            std::cout << "Online depth create mappoints!!!!!!!!  " << nPoints
                                      << std::endl;
                        }
                    }
                }
            }
        }

        if (enable_ground_height_scale)
        {
            float img_width = float(mpMap->img_width);
            float img_height = float(mpMap->img_height);

            // do it in every frame, otherwise may take longer time when do it together for many frames.
            for (size_t iMP = 0; iMP < mCurrentFrame.mvpMapPoints.size(); iMP++)
                if (pKF->mvKeysUn[iMP].pt.x > img_width / ground_roi_middle &&
                    pKF->mvKeysUn[iMP].pt.x < img_width / ground_roi_middle * (ground_roi_middle - 1))
                    if (pKF->mvKeysUn[iMP].pt.y >
                        img_height / ground_roi_lower *
                        (ground_roi_lower -
                         1)) // lower 1/3, I checked kitti sequence, roughly true.
                    {
                        bool not_in_object = true;
                        if (pKF->keypoint_inany_object.size() > 0)
                            if (pKF->keypoint_inany_object[iMP])
                                not_in_object = false;
                        if (not_in_object)
                            pKF->ground_region_potential_pts.push_back(
                                    iMP); // used for latter adjacent frame ground fitting
                    }

            if (pKF->mnId % ground_everyKFs == 0)
            {
                unsigned long anchor_frame_kfid = 0;
                if (int(pKF->mnId) > ground_everyKFs)
                    anchor_frame_kfid = pKF->mnId - ground_everyKFs;
                KeyFrame *first_keyframe = nullptr;
                std::vector<KeyFrame *> ground_local_KFs;
                unsigned long minKFid = pKF->mnId;
                for (size_t ii = 0; ii < mvpLocalKeyFrames.size(); ii++)
                {
                    KeyFrame *pKFi = mvpLocalKeyFrames[ii];
                    if (pKFi->mnId >= anchor_frame_kfid)
                    {
                        ground_local_KFs.push_back(pKFi);
                        pKFi->mnGroundFittingForKF = pKF->mnId;
                        if (pKFi->mnId < minKFid)
                        {
                            minKFid = pKFi->mnId; // the original anchor frame id might not exist due to
                            // culling.
                            first_keyframe = pKFi;
                        }
                    }
                }
                if (first_keyframe == nullptr)
                {
                    ROS_ERROR_STREAM("Not found first keyframe!!!  ");
                    exit(0);
                }
                ground_local_KFs.push_back(pKF);
                anchor_frame_kfid = pKF->mnId;
                int initializer_starting_frame_id =
                        (*mlpReferences.begin())->mnFrameId; // a fixed value

                KeyFrame *median_keyframe = nullptr; // scale relative to the center frames instead of
                // the begining frame? more accurate?
                if (height_esti_history.size() > 0)
                {
                    vector<unsigned> range_kf_ids;
                    for (size_t i = 0; i < ground_local_KFs.size(); i++)
                        range_kf_ids.push_back(ground_local_KFs[i]->mnId);
                    sort(range_kf_ids.begin(), range_kf_ids.end());
                    unsigned median_frameid = range_kf_ids[range_kf_ids.size() / 2];
                    for (size_t i = 0; i < ground_local_KFs.size(); i++)
                        if (ground_local_KFs[i]->mnId == median_frameid)
                        {
                            median_keyframe = ground_local_KFs[i];
                            break;
                        }
                    if (median_keyframe == nullptr)
                    {
                        ROS_ERROR_STREAM("Not found median keyframe!!!  ");
                        exit(0);
                    }
                }
                else
                    median_keyframe = first_keyframe; // still want to scale at the very first fram

                bool recently_have_object = false;
                if (recently_have_object)
                    ROS_ERROR_STREAM("Found cuboid landmark in this range");
                if ((!recently_have_object) || (height_esti_history.size() < 1))
                {
                    pcl::PointCloud<pcl::PointXYZ> cloud;
                    pcl::PointXYZ pt;
                    cloud.points.reserve(mvpLocalMapPoints.size());
                    vector<MapPoint *> potential_plane_points;
                    for (size_t i = 0; i < ground_local_KFs.size(); i++)
                    {
                        std::vector<MapPoint *> framepointmatches =
                                ground_local_KFs[i]->GetMapPointMatches();
                        for (size_t j = 0; j < ground_local_KFs[i]->ground_region_potential_pts.size();
                             j++)
                        {
                            MapPoint *pMP =
                                    framepointmatches[ground_local_KFs[i]->ground_region_potential_pts[j]];
                            if (pMP)
                                if (!pMP->isBad())
                                    if (pMP->mnGroundFittingForKF != pKF->mnId)
                                    {
                                        cv::Mat point_position =
                                                pMP->GetWorldPos(); // fit plane in global frame, then
                                        // tranform plane. saving time for point
                                        // transformation
                                        pt.x = point_position.at<float>(0);
                                        pt.y = point_position.at<float>(1);
                                        pt.z = point_position.at<float>(2);
                                        cloud.points.push_back(pt);
                                        potential_plane_points.push_back(pMP);
                                        pMP->mnGroundFittingForKF = pKF->mnId;
                                    }
                        }
                    }
                    std::cout << "Potential plane pt size    " << potential_plane_points.size() << "   "
                              << ground_local_KFs.size() << std::endl;

                    // TODO can we directly search height plane to find points supporting it?? not using
                    // ransac. Song used it.
                    pcl::SACSegmentation<pcl::PointXYZ> *seg =
                            new pcl::SACSegmentation<pcl::PointXYZ>();
                    seg->setOptimizeCoefficients(true);
                    seg->setModelType(pcl::SACMODEL_PLANE);
                    seg->setMethodType(pcl::SAC_RANSAC);
                    if (height_esti_history.size() > 0)
                        seg->setDistanceThreshold(ground_dist_ratio * height_esti_history.back());
                    else
                        seg->setDistanceThreshold(0.005); // the raw map is scaled to mean 1.
                    pcl::ModelCoefficients coefficients;
                    pcl::PointIndices inliers;
                    seg->setInputCloud(cloud.makeShared());
                    seg->segment(inliers, coefficients);

                    Eigen::Vector4f global_fitted_plane(coefficients.values[0], coefficients.values[1],
                                                        coefficients.values[2], coefficients.values[3]);
                    float cam_plane_dist, angle_diff_normal;

                    // transform to anchor frame
                    KeyFrame *anchor_frame = first_keyframe; // first_keyframe  median_keyframe  pKF;
                    cv::Mat anchor_Tcw = anchor_frame->GetPose();
                    cv::Mat anchor_Twc = anchor_frame->GetPoseInverse();

                    // take averge of all camera pose dist to plane,  not just wrt anchor frame
                    if (1)
                    {
                        float sum_cam_dist = 0;
                        float sum_angle_diff = 0;
                        vector<float> temp_dists;
                        for (size_t i = 0; i < ground_local_KFs.size(); i++)
                        {
                            KeyFrame *localkf = ground_local_KFs[i];
                            cv::Mat cam_Twc = localkf->GetPoseInverse();
                            Eigen::Matrix4f cam_Twc_eig = Converter::toMatrix4f(cam_Twc);
                            Eigen::Vector4f local_kf_plane =
                                    cam_Twc_eig.transpose() * global_fitted_plane;
                            local_kf_plane = local_kf_plane /
                                             local_kf_plane.head<3>().norm(); // normalize the plane.

                            float local_cam_plane_dist = fabs(local_kf_plane(3));
                            float local_angle_diff_normal =
                                    acos(local_kf_plane.head(3).dot(Vector3f(0, 1, 0))) * 180.0 /
                                    M_PI; // 0~pi
                            if (local_angle_diff_normal > 90)
                                local_angle_diff_normal = 180.0 - local_angle_diff_normal;
                            sum_cam_dist += local_cam_plane_dist;
                            sum_angle_diff += local_angle_diff_normal;
                            temp_dists.push_back(local_cam_plane_dist);
                        }
                        cam_plane_dist = sum_cam_dist / float(ground_local_KFs.size());
                        angle_diff_normal = sum_angle_diff / float(ground_local_KFs.size());
                    }

                    ROS_WARN_STREAM("Find init plane  dist   " << cam_plane_dist << "   angle  "
                                                               << angle_diff_normal << "   inliers  "
                                                               << inliers.indices.size());

                    if (int(inliers.indices.size()) > ground_inlier_pts) // or ratio
                    {
                        if (angle_diff_normal < 10)
                        {
                            // for kitti 02, unstale initialization. needs more times
                            if ((fabs(cam_plane_dist - nominal_ground_height) < 0.6) ||
                                (height_esti_history.size() < 4)) // or compare with last time?
                            {
                                height_esti_history.push_back(cam_plane_dist);

                                if (height_esti_history.size() == 1)
                                {
                                    first_absolute_scale_frameid = first_keyframe->mnFrameId;
                                    first_absolute_scale_framestamp = first_keyframe->mTimeStamp;
                                }

                                for (size_t i = 0; i < inliers.indices.size(); i++)
                                    potential_plane_points[inliers.indices[i]]->ground_fitted_point =
                                            true;

                                float final_filter_height = cam_plane_dist;
                                // take average or recent tow/three frames. or median filter? is this
                                // correct if object scale???
                                if (height_esti_history.size() > 2)
                                {
                                    final_filter_height =
                                            0.6 * height_esti_history.back() + 0.4 * filtered_ground_height;
                                }
                                filtered_ground_height = final_filter_height;

                                float scaling_ratio = nominal_ground_height / final_filter_height;
                                if (height_esti_history.size() > 1) // ignore the first time.
                                {
                                    // don't want too large scaling, which might be wrong...
                                    scaling_ratio = std::min(std::max(scaling_ratio, 0.7f), 1.3f);
                                }
                                ROS_WARN_STREAM("Actually scale map and frames~~~~~~~~~~~~~~~~");

                                if (enable_ground_height_scale)
                                {
                                    for (size_t iMP = 0; iMP < mvpLocalMapPoints.size();
                                         iMP++) // approximatedly. actually mvpLocalMapPoints has much
                                        // more points
                                    {
                                        cv::Mat local_pt = anchor_Tcw.rowRange(0, 3).colRange(0, 3) *
                                                           mvpLocalMapPoints[iMP]->GetWorldPos() +
                                                           anchor_Tcw.rowRange(0, 3).col(3);
                                        cv::Mat scaled_global_pt =
                                                anchor_Twc.rowRange(0, 3).colRange(0, 3) *
                                                (local_pt * scaling_ratio) +
                                                anchor_Twc.rowRange(0, 3).col(3);
                                        mvpLocalMapPoints[iMP]->SetWorldPos(scaled_global_pt);
                                    }
                                    for (size_t iKF = 0; iKF < ground_local_KFs.size(); iKF++)
                                    {
                                        cv::Mat anchor_to_pose =
                                                ground_local_KFs[iKF]->GetPose() * anchor_Twc;
                                        anchor_to_pose.col(3).rowRange(0, 3) =
                                                anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
                                        ground_local_KFs[iKF]->SetPose(anchor_to_pose * anchor_Tcw);
                                    }
                                    cv::Mat anchor_to_pose = mLastFrame.mTcw * anchor_Twc;
                                    anchor_to_pose.col(3).rowRange(0, 3) =
                                            anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
                                    mLastFrame.SetPose(anchor_to_pose * anchor_Tcw);
                                    mCurrentFrame.SetPose(pKF->GetPose());
                                    mVelocity.col(3).rowRange(0, 3) =
                                            mVelocity.col(3).rowRange(0, 3) * scaling_ratio;

                                    // loop over mlpReferences, if any frames' references frames lie in
                                    // this range, scale the relative poses accordingly mlpReferences
                                    // doesn't include the initialization stage... // if it is bad...??
                                    for (size_t ind =
                                            first_keyframe->mnFrameId - initializer_starting_frame_id;
                                         ind < mlpReferences.size(); ind++)
                                        if (mlpReferences[ind]->mnGroundFittingForKF == pKF->mnId)
                                        {
                                            cv::Mat Tcr =
                                                    mlRelativeFramePoses[ind]; // reference to current
                                            Tcr.col(3).rowRange(0, 3) =
                                                    Tcr.col(3).rowRange(0, 3) * scaling_ratio;
                                            mlRelativeFramePoses[ind] = Tcr;
                                        }
                                }
                            }
                            else
                                ROS_ERROR_STREAM("Too large change compared to last time.  "
                                                         << cam_plane_dist << "   last  "
                                                         << filtered_ground_height);
                        }
                        else
                            ROS_ERROR_STREAM("Bad ground orientation.");
                    }
                    else
                        ROS_ERROR_STREAM("Not enough inliers.");
                }
            }
        }

        pKF->mvDynamicArea = mCurrentFrame.mvDynamicArea;
        mpLocalMapper->InsertKeyFrame(pKF);
        mpLocalMapper->SetNotStop(false);

        double minT, maxT;
        cv::minMaxIdx(mImDepth, &minT, &maxT);

        cout << "Depth mat type " << mImDepth.type() << endl;
        cout << "DepthMapFactor" << mDepthMapFactor << endl;
        cout << "BEFORE +++ (" << minT << ", " << maxT << ") ";

        mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SearchLocalPoints() {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP) {
                if (pMP->isBad()) {
                    *vit = static_cast<MapPoint *>(NULL);
                } else {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                    pMP->mbTrackInViewR = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
                pMP->IncreaseVisible();
                nToMatch++;
            }
            if (pMP->mbTrackInView) {
                mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
            }
        }

        if (nToMatch > 0) {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
                th = 3;
            if (mpAtlas->isImuInitialized()) {
                if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
                    th = 2;
                else
                    th = 6;
            } else if (!mpAtlas->isImuInitialized() &&
                       (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                        mSensor == System::IMU_RGBD)) {
                th = 10;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;

            if (mState == LOST || mState == RECENTLY_LOST) // Lost for less than 1 second
                th = 15; // 15

            int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints,
                                                     mpLocalMapper->mThFarPoints);
        }
    }

    void Tracking::UpdateLocalMap() {
        // This is for visualization
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints() {
        mvpLocalMapPoints.clear();

        int count_pts = 0;

        for (vector<KeyFrame *>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(), itEndKF = mvpLocalKeyFrames.rend();
             itKF != itEndKF; ++itKF) {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
                 itMP != itEndMP; itMP++) {

                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad()) {
                    count_pts++;
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }


    void Tracking::UpdateLocalKeyFrames() {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
            for (int i = 0; i < mCurrentFrame.N; i++) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP) {
                    if (!pMP->isBad()) {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end();
                             it != itend; it++)
                            keyframeCounter[it->first]++;
                    } else {
                        mCurrentFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        } else {
            for (int i = 0; i < mLastFrame.N; i++) {
                // Using lastframe since current frame has not matches yet
                if (mLastFrame.mvpMapPoints[i]) {
                    MapPoint *pMP = mLastFrame.mvpMapPoints[i];
                    if (!pMP)
                        continue;
                    if (!pMP->isBad()) {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end();
                             it != itend; it++)
                            keyframeCounter[it->first]++;
                    } else {
                        // MODIFICATION
                        mLastFrame.mvpMapPoints[i] = NULL;
                    }
                }
            }
        }


        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
             it != itEnd; it++) {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max) {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(pKF);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80) // 80
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
                 itNeighKF != itEndNeighKF; itNeighKF++) {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad()) {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad()) {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent) {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // Add 10 last temporal KFs (mainly for IMU)
        if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
            mvpLocalKeyFrames.size() < 80) {
            KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

            const int Nd = 20;
            for (int i = 0; i < Nd; i++) {
                if (!tempKeyFrame)
                    break;
                if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(tempKeyFrame);
                    tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    tempKeyFrame = tempKeyFrame->mPrevKF;
                }
            }
        }

        if (pKFmax) {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization() {
        Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame,
                                                                                         mpAtlas->GetCurrentMap());

        if (vpCandidateKFs.empty()) {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<MLPnPsolver *> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++) {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else {
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15) {
                    vbDiscarded[i] = true;
                    continue;
                } else {
                    MLPnPsolver *pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991);  //This solver needs at least 6 points
                    vpMLPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch) {
            for (int i = 0; i < nKFs; i++) {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                MLPnPsolver *pSolver = vpMLPnPsolvers[i];
                Eigen::Matrix4f eigTcw;
                bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore) {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (bTcw) {
                    Sophus::SE3f Tcw(eigTcw);
                    mCurrentFrame.SetPose(Tcw);
                    // Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++) {
                        if (vbInliers[j]) {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        } else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50) {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10,
                                                                      100);

                        if (nadditional + nGood >= 50) {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50) {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3,
                                                                          64);

                                // Final optimization
                                if (nGood + nadditional >= 50) {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50) {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch) {
            return false;
        } else {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            cout << "Relocalized!!" << endl;
            return true;
        }

    }

    void Tracking::Reset(bool bLocMap) {
        Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

        if (mpViewer) {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        if (!bLocMap) {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
            mpLocalMapper->RequestReset();
            Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        }


        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clear();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearAtlas();
        mpAtlas->CreateNewMap();
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
            mpAtlas->SetInertialSensor();
        mnInitialFrameId = 0;

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        mbReadyToInitializate = false;
        mbSetInit = false;

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();
        mCurrentFrame = Frame();
        mnLastRelocFrameId = 0;
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    void Tracking::ResetActiveMap(bool bLocMap) {
        Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
        if (mpViewer) {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        Map *pMap = mpAtlas->GetCurrentMap();

        if (!bLocMap) {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
            mpLocalMapper->RequestResetActiveMap(pMap);
            Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
        }

        // Reset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearMap();


        //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
        //Frame::nNextId = mnLastInitFrameId;
        mnLastInitFrameId = Frame::nNextId;
        //mnLastRelocFrameId = mnLastInitFrameId;
        mState = NO_IMAGES_YET; //NOT_INITIALIZED;

        mbReadyToInitializate = false;

        list<bool> lbLost;
        // lbLost.reserve(mlbLost.size());
        unsigned int index = mnFirstFrameId;
        cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
        for (Map *pMap: mpAtlas->GetAllMaps()) {
            if (pMap->GetAllKeyFrames().size() > 0) {
                if (index > pMap->GetLowerKFID())
                    index = pMap->GetLowerKFID();
            }
        }

        //cout << "First Frame id: " << index << endl;
        int num_lost = 0;
        cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

        for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++) {
            if (index < mnInitialFrameId)
                lbLost.push_back(*ilbL);
            else {
                lbLost.push_back(true);
                num_lost += 1;
            }

            index++;
        }
        cout << num_lost << " Frames set to lost" << endl;

        mlbLost = lbLost;

        mnInitialFrameId = mCurrentFrame.mnId;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mCurrentFrame = Frame();
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        mbVelocity = false;

        if (mpViewer)
            mpViewer->Release();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

    vector<MapPoint *> Tracking::GetLocalMapMPS() {
        return mvpLocalMapPoints;
    }

    void Tracking::ChangeCalibration(const string &strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        mK_.setIdentity();
        mK_(0, 0) = fx;
        mK_(1, 1) = fy;
        mK_(0, 2) = cx;
        mK_(1, 2) = cy;

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag) {
        mbOnlyTracking = flag;
    }

    void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame) {
        Map *pMap = pCurrentKeyFrame->GetMap();
        unsigned int index = mnFirstFrameId;
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin();
        list<bool>::iterator lbL = mlbLost.begin();
        for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lbL++) {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            while (pKF->isBad()) {
                pKF = pKF->GetParent();
            }

            if (pKF->GetMap() == pMap) {
                (*lit).translation() *= s;
            }
        }

        mLastBias = b;

        mpLastKeyFrame = pCurrentKeyFrame;

        mLastFrame.SetNewBias(mLastBias);
        mCurrentFrame.SetNewBias(mLastBias);

        while (!mCurrentFrame.imuIsPreintegrated()) {
            usleep(500);
        }


        if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
            mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                          mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                          mLastFrame.mpLastKeyFrame->GetVelocity());
        } else {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
            float t12 = mLastFrame.mpImuPreintegrated->dT;

            mLastFrame.SetImuPoseVelocity(
                    IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                    twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                    Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                    Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        if (mCurrentFrame.mpImuPreintegrated) {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

            const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
            float t12 = mCurrentFrame.mpImuPreintegrated->dT;

            mCurrentFrame.SetImuPoseVelocity(
                    IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                    twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                    Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                    Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
        }

        mnFirstImuFrameId = mCurrentFrame.mnId;
    }

    void Tracking::NewDataset() {
        mnNumDataset++;
    }

    int Tracking::GetNumberDataset() {
        return mnNumDataset;
    }

    int Tracking::GetMatchesInliers() {
        return mnMatchesInliers;
    }

    void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder) {
        mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
        //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
    }

    void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap) {
        mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
        if (!strNameFile_kf.empty())
            mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
    }

    float Tracking::GetImageScale() {
        return mImageScale;
    }

#ifdef REGISTER_LOOP
    void Tracking::RequestStop()
{
unique_lock<mutex> lock(mMutexStop);
mbStopRequested = true;
}

bool Tracking::Stop()
{
unique_lock<mutex> lock(mMutexStop);
if(mbStopRequested && !mbNotStop)
{
mbStopped = true;
cout << "Tracking STOP" << endl;
return true;
}

return false;
}

bool Tracking::stopRequested()
{
unique_lock<mutex> lock(mMutexStop);
return mbStopRequested;
}

bool Tracking::isStopped()
{
unique_lock<mutex> lock(mMutexStop);
return mbStopped;
}

void Tracking::Release()
{
unique_lock<mutex> lock(mMutexStop);
mbStopped = false;
mbStopRequested = false;
}
#endif

    void Tracking::SetDetector(YoloDetection* pDetector)
    {
        mpDetector = pDetector;
    }

} //namespace ORB_SLAM
