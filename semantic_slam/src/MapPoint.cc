
#include "MapPoint.h"
#include "ORBmatcher.h"

// For 3D cuboid tetsing (optimize)
#include "MapCuboidObject.h"
#include "Converter.h"
#include "Parameter.h"

#include <mutex>

namespace semantic_slam {

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map *pMap) :
            mnFirstKFid(FirstKFid), mnFirstFrame(FirstFrame), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap) {
        Pos.copyTo(semantic_slam::Converter::toCvMat(mWorldPos));
        mNormalVector = semantic_slam::Converter::toVector3f(cv::Mat::zeros(3, 1, CV_32F));

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

        // For 3D Cuboid (optimize)
        max_object_vote = 0;
        best_object = nullptr;
        mnGroundFittingForKF = 0;
        already_bundled = false;

        PosToObj = cv::Mat::zeros(3, 1, CV_32F);
    }

    MapPoint::MapPoint() :
            mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)) {
        mpReplaced = static_cast<MapPoint *>(NULL);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
            mnOriginMapId(pMap->GetId()) {
        SetWorldPos(Pos);

        mNormalVector.setZero();

        mbTrackInViewR = false;
        mbTrackInView = false;

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

        // For 3D Cuboid (optimize)
        max_object_vote = 0;
        best_object = nullptr;
        mnGroundFittingForKF = 0;
        already_bundled = false;

        PosToObj = cv::Mat::zeros(3, 1, CV_32F);
    }

    MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF, KeyFrame *pHostKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
            mnOriginMapId(pMap->GetId()) {
        mInvDepth = invDepth;
        mInitU = (double) uv_init.x;
        mInitV = (double) uv_init.y;
        mpHostKF = pHostKF;

        mNormalVector.setZero();

        // Worldpos is not set
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

        // For 3D Cuboid (optimize)
        max_object_vote = 0;
        best_object = nullptr;
        mnGroundFittingForKF = 0;
        already_bundled = false;

        PosToObj = cv::Mat::zeros(3, 1, CV_32F);
    }

    MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF) :
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId()) {
        SetWorldPos(Pos);

        Eigen::Vector3f Ow;
        if (pFrame->Nleft == -1 || idxF < pFrame->Nleft) {
            Ow = pFrame->GetCameraCenter();
        } else {
            Eigen::Matrix3f Rwl = pFrame->GetRwc();
            Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
            Eigen::Vector3f twl = pFrame->GetOw();

            Ow = Rwl * tlr + twl;
        }
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / mNormalVector.norm();

        Eigen::Vector3f PC = mWorldPos - Ow;
        const float dist = PC.norm();
        const int level = (pFrame->Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
                                                : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                                                         : pFrame->mvKeysRight[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

        // For 3D Cuboid (optimize)
//    max_object_vote = 0;
//    best_object = nullptr;
//    mnGroundFittingForKF = 0;
//    already_bundled = false;
//
//    PosToObj = cv::Mat::zeros(3, 1, CV_32F);
    }

    void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
    }

    Eigen::Vector3f MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    Eigen::Vector3f MapPoint::GetNormal() {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector;
    }


    KeyFrame *MapPoint::GetReferenceKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::SetReferenceKeyFrame(KeyFrame *pRefKF) {
        mpRefKF = pRefKF;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        tuple<int, int> indexes;

        if (mObservations.count(pKF)) {
            indexes = mObservations[pKF];
        } else {
            indexes = tuple<int, int>(-1, -1);
        }

        if (pKF->NLeft != -1 && idx >= pKF->NLeft) {
            get<1>(indexes) = idx;
        } else {
            get<0>(indexes) = idx;
        }

        mObservations[pKF] = indexes;

        if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0)
            nObs += 2;
        else
            nObs++;

        // For 3D Cuboid testing (optimize)
        // NOTE points usually initially associated different objects, due to occlusions
        if (associate_point_with_object) {
            if (use_dynamic_klt_features && is_dynamic) {
                if (pKF->keypoint_associate_objectID_harris.size() > 0) {
                    int frame_cubod_id = pKF->keypoint_associate_objectID_harris[idx];
                    if (frame_cubod_id > -1)
                        AddObjectObservation(pKF->local_cuboids[frame_cubod_id]);
                }
            } else {
                if (pKF->keypoint_associate_objectID.size() > 0) {
                    int frame_cubod_id = pKF->keypoint_associate_objectID[idx];
                    if (frame_cubod_id > -1)
                    {
//                        std::cout << "[MapPoint] : " << frame_cubod_id << std::endl;
                        AddObjectObservation(pKF->local_cuboids[frame_cubod_id]);
                    }
                }
            }
        }
    }

    void MapPoint::EraseObservation(KeyFrame *pKF) {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF)) {
                tuple<int, int> indexes = mObservations[pKF];
                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                if (leftIndex != -1) {
                    if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
                        nObs -= 2;
                    else
                        nObs--;
                }
                if (rightIndex != -1) {
                    nObs--;
                }

                mObservations.erase(pKF);

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();
    }


    std::map<KeyFrame *, std::tuple<int, int>> MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag() {
        map<KeyFrame *, tuple<int, int>> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            int leftIndex = get<0>(mit->second), rightIndex = get<1>(mit->second);
            if (leftIndex != -1) {
                pKF->EraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1) {
                pKF->EraseMapPointMatch(rightIndex);
            }
        }

        mpMap->EraseMapPoint(this);
    }

    MapPoint *MapPoint::GetReplaced() {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void MapPoint::Replace(MapPoint *pMP) {
        if (pMP->mnId == this->mnId)
            return;

        int nvisible, nfound;
        map<KeyFrame *, tuple<int, int>> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;

            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (!pMP->IsInKeyFrame(pKF)) {
                if (leftIndex != -1) {
                    pKF->ReplaceMapPointMatch(leftIndex, pMP);
                    pMP->AddObservation(pKF, leftIndex);
                }
                if (rightIndex != -1) {
                    pKF->ReplaceMapPointMatch(rightIndex, pMP);
                    pMP->AddObservation(pKF, rightIndex);
                }
            } else {
                if (leftIndex != -1) {
                    pKF->EraseMapPointMatch(leftIndex);
                }
                if (rightIndex != -1) {
                    pKF->EraseMapPointMatch(rightIndex);
                }
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad() {
        unique_lock<mutex> lock1(mMutexFeatures, std::defer_lock);
        unique_lock<mutex> lock2(mMutexPos, std::defer_lock);
        lock(lock1, lock2);

        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors() {
        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;

        map<KeyFrame *, tuple<int, int>> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if (mbBad)
                return;
            observations = mObservations;
        }

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;

            if (!pKF->isBad()) {
                tuple<int, int> indexes = mit->second;
                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                if (leftIndex != -1) {
                    vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
                }
                if (rightIndex != -1) {
                    vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
                }
            }
        }

        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return tuple<int, int>(-1, -1);
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth() {
        map<KeyFrame *, tuple<int, int>> observations;
        KeyFrame *pRefKF;
        Eigen::Vector3f Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos;
        }

        if (observations.empty())
            return;

        Eigen::Vector3f normal;
        normal.setZero();
        int n = 0;
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;

            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1) {
                Eigen::Vector3f Owi = pKF->GetCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                normal = normal + normali / normali.norm();
                n++;
            }
            if (rightIndex != -1) {
                Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
                Eigen::Vector3f normali = Pos - Owi;
                normal = normal + normali / normali.norm();
                n++;
            }
        }

        Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
        const float dist = PC.norm();

        tuple<int, int> indexes = observations[pRefKF];
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        int level;
        if (pRefKF->NLeft == -1) {
            level = pRefKF->mvKeysUn[leftIndex].octave;
        } else if (leftIndex != -1) {
            level = pRefKF->mvKeys[leftIndex].octave;
        } else {
            level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
        }

        //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    void MapPoint::SetNormalVector(const Eigen::Vector3f &normal) {
        unique_lock<mutex> lock3(mMutexPos);
        mNormalVector = normal;
    }

    float MapPoint::GetMinDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, Frame *pF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

    int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor) {
        float ratio;
        {
            unique_lock<mutex> lock3(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        return ceil(log(ratio) / logScaleFactor);
    }

    void MapPoint::PrintObservations() {
        cout << "MP_OBS: MP " << mnId << endl;
        for (map<KeyFrame *, tuple<int, int>>::iterator mit = mObservations.begin(), mend = mObservations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;
            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
            cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
        }
    }

    Map *MapPoint::GetMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void MapPoint::UpdateMap(Map *pMap) {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }

    void MapPoint::PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP) {
        mBackupReplacedId = -1;
        if (mpReplaced && spMP.find(mpReplaced) != spMP.end())
            mBackupReplacedId = mpReplaced->mnId;

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();

        // Save the id and position in each KF who view it
        std::map<KeyFrame *, std::tuple<int, int> > tmp_mObservations;
        tmp_mObservations.insert(mObservations.begin(), mObservations.end());

        for (std::map<KeyFrame *, std::tuple<int, int> >::const_iterator it = tmp_mObservations.begin(), end = tmp_mObservations.end();
             it != end; ++it) {
            KeyFrame *pKFi = it->first;
            if (spKF.find(pKFi) != spKF.end()) {
                mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
                mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
            } else {
                EraseObservation(pKFi);
            }
        }

        // Save the id of the reference KF
        if (spKF.find(mpRefKF) != spKF.end()) {
            mBackupRefKFId = mpRefKF->mnId;
        }
    }

    void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid) {
        mpRefKF = mpKFid[mBackupRefKFId];
        if (!mpRefKF) {
            cout << "ERROR: MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
        }
        mpReplaced = static_cast<MapPoint *>(NULL);
        if (mBackupReplacedId >= 0) {
            map<long unsigned int, MapPoint *>::iterator it = mpMPid.find(mBackupReplacedId);
            if (it != mpMPid.end())
                mpReplaced = it->second;
        }

        mObservations.clear();

        for (map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end();
             it != end; ++it) {
            KeyFrame *pKFi = mpKFid[it->first];
            map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
            std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
            if (pKFi) {
                mObservations[pKFi] = indexes;
            }
        }

        mBackupObservationsId1.clear();
        mBackupObservationsId2.clear();
    }

    cv::Mat MapPoint::GetWorldPosBA() {
        unique_lock<mutex> lock(mMutexPos);
        if (is_dynamic && best_object != nullptr) {
            return Converter::toCvMat(best_object->GetWorldPosBA().pose.map(Converter::toVector3d(PosToObj)));
        }

        return cv::Mat::zeros(0, 0, CV_32F); // should not happen, this function only for dynamic point
    }

    Eigen::Vector3f MapPoint::GetWorldPosVec() {
        cv::Mat worldpose = Converter::toCvMat(GetWorldPos());
        return Eigen::Vector3f(worldpose.at<float>(0), worldpose.at<float>(1), worldpose.at<float>(2));
    }

    void MapPoint::AddObjectObservation(MapCuboidObject *obj) {
        // unique_lock<mutex> lock(mMutexObject);
        if (obj->already_associated)
        {
            MapCuboidObject *objlandmark = obj->associated_landmark;

            if (MapObjObservations.count(objlandmark))
                MapObjObservations[objlandmark]++;
            else
                MapObjObservations[objlandmark] = 1;

            // update best object, no need to call FindBestObject()
            if (MapObjObservations[objlandmark] > max_object_vote) {
                if ((best_object != nullptr) && (best_object != objlandmark))
                    best_object->EraseUniqueMapPoint(this, max_object_vote);

                best_object = objlandmark;

                max_object_vote = MapObjObservations[objlandmark];
                best_object->AddUniqueMapPoint(this,max_object_vote); // even being added before, still increase observation num
            }
        } else {
            LocalObjObservations.insert(obj);
            obj->AddPotentialMapPoint(this); // for frame local object, still add point observation. will be useful when local object changes to landmark.
        }
    }

    void MapPoint::EraseObjectObservation(MapCuboidObject *obj) {
        unique_lock<mutex> lock(mMutexObject);
        if (MapObjObservations.count(obj)) {
            MapObjObservations[obj]--;
            FindBestObject();
        }
    }

    void MapPoint::FindBestObject() {
        // for loop to update best beloned object, only for object landmarks, frame local object is not considered.
        best_object = nullptr;
        max_object_vote = 0;
        for (map<MapCuboidObject *, int>::iterator mit = MapObjObservations.begin(), mend = MapObjObservations.end();
             mit != mend; mit++)
            if (mit->second > max_object_vote) {
                max_object_vote = mit->second;
                best_object = mit->first;
            }
    }

    int MapPoint::GetBelongedObject(MapCuboidObject *&obj) {
        unique_lock<mutex> lock(mMutexObject);
        obj = best_object;
        return max_object_vote;
    }

    MapCuboidObject *MapPoint::GetBelongedObject() {
        unique_lock<mutex> lock(mMutexObject);
        return best_object;
    }

} //namespace semantic_slam
