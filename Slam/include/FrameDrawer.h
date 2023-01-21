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


#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Atlas.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>


namespace ORB_SLAM3
{

class Tracking;
class Viewer;
class MapPoint;
class MapObject;
class Map;

class FrameDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas* pAtlas);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(float imageScale=1.f);
    cv::Mat DrawRightFrame(float imageScale=1.f);

    bool both;

protected:

    // detect 3d cuboid
    std::vector<cv::Rect> bbox_2ds;                // yolo detected 2D bbox_2d, which has 3D cuboid
    std::vector<Eigen::Matrix2Xi> box_corners_2ds; //2*8 corners on object.
    std::vector<Eigen::MatrixXi> edge_markers_2ds; // in order to plot 2d cuboids with 8 corners.
    std::vector<int> truth2d_trackid;              // truth 2d track id

    std::vector<int> potential_ground_fit_inds;
    int current_frame_id = -1;
    int saved_img_id = -1;

    // just for debug visualization
    std::vector<cv::KeyPoint> mvCurrentKeys_inlastframe;
    std::vector<cv::Point2f> mvfeaturesklt_lastframe;
    std::vector<cv::Point2f> mvfeaturesklt_thisframe;

    std::vector<cv::Scalar> box_colors; // fixed
    std::vector<int> point_Object_AssoID;
    bool whether_keyframe;
    cv::Mat cam_pose_cw; // camera pose, world to cam
    Eigen::Matrix3d Kalib;

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys,mvCurrentKeysRight;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    std::vector<float> mvCurrentDepth;
    float mThDepth;

    Atlas* mpAtlas;

    std::mutex mMutex;
    vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

    Frame mCurrentFrame;
    vector<MapPoint*> mvpLocalMap;
    vector<cv::KeyPoint> mvMatchedKeys;
    vector<MapPoint*> mvpMatchedMPs;
    vector<cv::KeyPoint> mvOutlierKeys;
    vector<MapPoint*> mvpOutlierMPs;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
