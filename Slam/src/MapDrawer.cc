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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#include "Converter.h"
#include "MapObject.h"
#include "Config.h"
#include "detect_3d_cuboid/object_3d_util.h"

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    box_colors.push_back(Vector3f(230, 0, 0) / 255.0);     // red  0
    box_colors.push_back(Vector3f(60, 180, 75) / 255.0);   // green  1
    box_colors.push_back(Vector3f(0, 0, 255) / 255.0);     // blue  2
    box_colors.push_back(Vector3f(255, 0, 255) / 255.0);   // Magenta  3
    box_colors.push_back(Vector3f(255, 165, 0) / 255.0);   // orange 4
    box_colors.push_back(Vector3f(128, 0, 128) / 255.0);   // purple 5
    box_colors.push_back(Vector3f(0, 255, 255) / 255.0);   // cyan 6
    box_colors.push_back(Vector3f(210, 245, 60) / 255.0);  // lime  7
    box_colors.push_back(Vector3f(250, 190, 190) / 255.0); // pink  8
    box_colors.push_back(Vector3f(0, 128, 128) / 255.0);   // Teal  9

    all_edge_pt_ids.resize(8, 2); // draw 8 edges except front face
    all_edge_pt_ids << 2, 3, 3, 4, 4, 1, 3, 7, 4, 8, 6, 7, 7, 8, 8, 5;
    all_edge_pt_ids.array() -= 1;
    front_edge_pt_ids.resize(4, 2);
    front_edge_pt_ids << 1, 2, 2, 6, 6, 5, 5, 1;
    front_edge_pt_ids.array() -= 1;
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    // draw common map points
    int shift_truth_map = 0;
    float normal_point_size = 1; // raw: 1

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    if(1)
    {
        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
            glVertex3f(pos(0),pos(1),pos(2));

        }

        glEnd();
    }

    if (enable_ground_height_scale) // draw ground point, better to show gray points for above.
    {
        glPointSize(mPointSize * normal_point_size * 2);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0); // red
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            if (vpMPs[i]->isBad())
                continue;
            if (vpMPs[i]->ground_fitted_point)
            {
                cv::Mat pos = vpMPs[i]->GetWorldPos();
                glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)); //+shift_truth_map
            }
        }
        glEnd();
    }

    // draw the object points
    if (whether_detect_object && associate_point_with_object)
    {
        glPointSize(mPointSize * 5);
        glBegin(GL_POINTS);
        const vector<MapObject *> all_Map_objs = mpMap->GetAllMapObjects();
        for (size_t object_id = 0; object_id < all_Map_objs.size(); object_id++)
        {
            MapObject *obj_landmark = all_Map_objs[object_id];
            if (obj_landmark->isBad())
                continue;
            vector<MapPoint *> owned_mappoints;
            owned_mappoints = obj_landmark->used_points_in_BA_filtered; // points really used in BA
            if (whether_dynamic_object)
                owned_mappoints = obj_landmark->GetUniqueMapPoints();
            Vector3f box_color = box_colors[obj_landmark->mnId % box_colors.size()];
            glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
            for (size_t pt_id = 0; pt_id < owned_mappoints.size(); pt_id++)
            {
                MapPoint *mpt = owned_mappoints[pt_id];
                if (!mpt->isBad())
                {
                    cv::Mat pos;
                    if (bundle_object_opti)
                    {
                        if (obj_landmark->obj_been_optimized)
                            pos = mpt->GetWorldPosBA();
                        else
                            continue;
                    }
                    else
                    {
                        pos = mpt->GetWorldPos();
                    }
                    if (pos.rows == 0)
                        continue;

                    if (mpt->is_dynamic)
                    {
                        if (mpt->Observations() < 2) // don't show just depth inited frame
                            continue;
                    }
                    glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                }
            }
        }
        glEnd();
    }
}

    void MapDrawer::DrawMapCuboids() // ideally this should be draw map cuboids.
    {
        // make sure final cuboid is in init world frame.
        // draw all map objects
        const vector<MapObject *> all_Map_objs = mpMap->GetAllMapObjects();
        Vector4d front_face_color(1.0, 0.0, 1.0, 1.0); // draw front face edges magenta

        for (size_t object_id = 0; object_id < all_Map_objs.size(); object_id++)
        {
            MapObject *obj_landmark = all_Map_objs[object_id];

            if (obj_landmark->isBad()) // some good, some bad, some not determined
                continue;

            // show objects that being optimized! for kitti fix scale, this will make map visualization
            // better.
            if (bundle_object_opti)
            {
                if (!obj_landmark->obj_been_optimized)
                {
                    continue;
                }
            }

            Eigen::MatrixXd cube_corners;
            if (bundle_object_opti && whether_dynamic_object)
                cube_corners = obj_landmark->pose_Twc_afterba
                        .compute3D_BoxCorner(); // show pose after BA, will have some delay,
                // but looks good
            else
                cube_corners = obj_landmark->GetWorldPos().compute3D_BoxCorner();

            if (obj_landmark->Observations() == 1)
            {
                glLineWidth(mGraphLineWidth * 2);
                glBegin(GL_LINES);
                front_face_color = Vector4d(0, 0, 128.0 / 255.0, 1.0);
            }
            else
            {
                glLineWidth(mGraphLineWidth * 4);
                glBegin(GL_LINES);
                front_face_color = Vector4d(1.0, 0.0, 1.0, 1.0);
            }
            // draw cuboid
            Vector3f box_color = box_colors[obj_landmark->mnId % box_colors.size()];
            glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
            for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++)
            {
                glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 0)),
                           cube_corners(1, all_edge_pt_ids(line_id, 0)),
                           cube_corners(2, all_edge_pt_ids(line_id, 0)));
                glVertex3f(cube_corners(0, all_edge_pt_ids(line_id, 1)),
                           cube_corners(1, all_edge_pt_ids(line_id, 1)),
                           cube_corners(2, all_edge_pt_ids(line_id, 1)));
            }
            for (int line_id = 0; line_id < front_edge_pt_ids.rows(); line_id++)
            {
                glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 0)),
                           cube_corners(1, front_edge_pt_ids(line_id, 0)),
                           cube_corners(2, front_edge_pt_ids(line_id, 0)));
                glVertex3f(cube_corners(0, front_edge_pt_ids(line_id, 1)),
                           cube_corners(1, front_edge_pt_ids(line_id, 1)),
                           cube_corners(2, front_edge_pt_ids(line_id, 1)));
            }
            glEnd();

            // draw dynamic object history path
            if (whether_dynamic_object && obj_landmark->is_dynamic &&
                obj_landmark->allDynamicPoses.size() > 0)
            {
                glLineWidth(mGraphLineWidth * 2);
                glBegin(GL_LINE_STRIP); // line strip connects adjacent points
                glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
                for (auto it = obj_landmark->allDynamicPoses.begin();
                     it != obj_landmark->allDynamicPoses.end(); it++)
                {
                    if (bundle_object_opti &&
                        !it->second.second) // only show optimized frame object pose
                        continue;
                    g2o::cuboid cubepose = it->second.first;
                    glVertex3f(cubepose.pose.translation()(0), cubepose.pose.translation()(1),
                               cubepose.pose.translation()(2));
                }
                glEnd();
            }
        }
    }


void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}
} //namespace ORB_SLAM
