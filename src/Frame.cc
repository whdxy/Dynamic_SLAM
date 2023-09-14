/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
int Frame::nLabelMin=-1; /// new
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


/// new
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,  const cv::Mat &imSemantic, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
    mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    //thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    //thread threadRight(&Frame::ExtractORB,this,1,imRight);
    thread threadLeft(&Frame::ExtractORBNew,this,0,imLeft,imSemantic);
    thread threadRight(&Frame::ExtractORBNew,this,1,imRight,imSemantic);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    //ComputeStereoMatches();
    ComputeStereoMatchesNew(imLeft, imRight);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}


void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::ExtractORBNew(int flag, const cv::Mat &im, const cv::Mat &imSemantic)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,imSemantic,mvKeys,mmKeysDynamic,mDescriptors,mmPixelsDynamic,mmBoundaryDynamic,true);
    else
        (*mpORBextractorRight)(im,imSemantic,mvKeysRight,mmKeysRightDynamic,mDescriptorsRight,mmPixelsDynamic,mmBoundaryDynamic,false);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        //cout << "vCandidates " << iL << ": " << vCandidates.size() << endl;
        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }
    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

/// new, 新增动态像素点双目匹配计算深度
void Frame::ComputeStereoMatchesNew(const cv::Mat &imLeft, const cv::Mat &imRight)
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    /// 将右目特征点加入备选项
    /// 第1行：32，45，67
    /// 第2行：2，4，6,10
    /// 第i行：...
    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        /// step1、用描述子作初匹配
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        /// step1、亚像素修正
        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }
    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }

    /// 上面是特征点双目匹配计算深度
    /// -----------------------
    /// 下面是像素点双目匹配计算深度
    const int L = 3; // 车辆深度半径
    const int w = 5;
    auto itPixels=mmPixelsDynamic.begin(), itPixelsend=mmPixelsDynamic.end();
    while(itPixels!=itPixelsend){
        int label=itPixels->first;
        const std::map<int, std::vector<int>>& m=itPixels->second;
        std::vector<int>& boundary = mmBoundaryDynamic[label];
        boundary.push_back(m.begin()->first);
        boundary.push_back((--m.end())->first);
        const int maskRows=boundary[3]-boundary[2];
        const int maskCols=boundary[1]-boundary[0];
        //cout << "uMin:" << boundary[0] << " uMax" << boundary[1] << " vMin:" << boundary[2] << " vMax" << boundary[3] << endl;
        //cout << "maskRows:" << maskRows << " maskCols" << maskCols << endl;

        if(maskRows<30 || maskCols <50){
            mmBoundaryDynamic.erase(mmBoundaryDynamic.find(label));
            itPixels = mmPixelsDynamic.erase(itPixels);
            itPixelsend = mmPixelsDynamic.end();
            if(itPixels==itPixelsend)
                break;
        }
        else{
            /// 1、整体匹配，求最佳偏移
            int bestdistMatch=INT_MAX;
            int bestUoffset=0;
            const int colStep=3;
            for(int i=0; i<boundary[0]; ){ // uMin左移动，v不变
                int dist=0;
                auto itm=m.begin(), itmend=m.end();
                for(; itm!=itmend; itm++){
                    int row=itm->first;
                    const vector<int>& vcol=itm->second;
                    for(int j=0; j<vcol.size(); j++){
                        if(vcol[j]-i>0)
                            dist+=abs(imRight.at<uchar>(row,vcol[j]-i)-imLeft.at<uchar>(row,vcol[j]));
                    }
                }

                if(bestdistMatch>dist){
                    bestdistMatch=dist;
                    bestUoffset=i;
                }
                i+=colStep;
            }

            //cv::Mat mask=imRight.rowRange(boundary[2],boundary[3]).colRange(boundary[0]-bestUoffset,boundary[1]-bestUoffset);
            //cv::imshow("mask", mask);
            //cv::waitKey(0);
            //cout << "uMin:" << boundary[0] << " uMax" << boundary[1] << " vMin:" << boundary[2] << " vMax" << boundary[3] << endl;
            //cout << "label:" << label << " bestdistMatch:" << bestdistMatch << " bestUoffset:" << bestUoffset << " uMin:" << boundary[0] << " uMax" << boundary[1] << " vMin:" << boundary[2] << " vMax" << boundary[3] << endl;
            cout << "label:" << label << " bestdistMatch:" << bestdistMatch << " bestUoffset:" << bestUoffset << endl;

            const int L = 3;
            const int w = 5;
            const float depthIniti = mbf/bestUoffset; /// 根据最佳偏移，计算最小和最大偏移
            const int maxUoffset = ceil(mbf/(depthIniti-L));
            const int minUoffset = floor(mbf/(depthIniti+L));

            auto itm=m.begin(), itmend=m.end();
            for(;itm!=itmend;itm++){ // 遍历像素点
                int rowL = itm->first;
                vector<int> v=itm->second;
                for(int i=0; i<v.size(); i++){
                    int colL=v[i];

                    if(rowL-w<0 || rowL+w>imLeft.rows || colL-w<0 || colL+w>imLeft.cols)
                        continue;

                    int bestDist=INT_MAX;
                    int bestURoffset=0;
                    //int bestDist=INT_MAX;
                    cv::Mat IL = imLeft.rowRange(rowL-w,rowL+w).colRange(colL-w, colL+w);
                    IL.convertTo(IL,CV_32F);
                    IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);
                    //cout << IL << endl;

                    for(int j=minUoffset; j<=maxUoffset; j++){

                        if(colL-w-j<0)
                            continue;

                        cv::Mat IR = imRight.rowRange(rowL-w,rowL+w).colRange(colL-w-j, colL+w-j);
                        IR.convertTo(IR,CV_32F);
                        IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);
                        //cout << IR << endl;

                        float dist = cv::norm(IL,IR,cv::NORM_L1);
                        //cout << "dist:" << dist << endl;
                        if(dist<bestDist)
                        {
                            bestDist = dist;
                            bestURoffset = j;
                        }
                    }
                    if(bestURoffset>=minD && bestURoffset<maxD)
                    {
                        if(bestURoffset<=0)
                        {

                        }
                        float d = mbf/bestURoffset;
                        //vDepth.push_back(d);
                    }
                    //cout << "   bestURoffset:" << bestURoffset << " d:" << (float)mbf/bestURoffset << endl;
                }
            }


            itPixels++;
        }
    }

    ///=================================================================
    auto itL = mmKeysDynamic.begin();
    //auto itR = mvKeysRightDynamic.begin();
    auto itend = mmKeysDynamic.end();
    const int minNum = 50; // 滤除小于50个像素点的动态物体


    bool boptial = false;
    /// 采用光流
    if(boptial){
        while(itL!=itend){
            if(itL->second.size()<minNum){
                itL = mmKeysDynamic.erase(itL);
                itend = mmKeysDynamic.end();
                if(itL==itend)
                    break;
            }
            else{
                int label = itL->first;
                std::vector<cv::KeyPoint> vkp = itL->second;
                std::vector<float> vDepth;
                int max_u=-1, max_v=-1;
                int min_u=INT_MAX, min_v=INT_MAX;
                vector<cv::Point2f> pt1, pt2;
                for(auto itkp=vkp.begin(),itkpend=vkp.end();  itkp!=itkpend; itkp++){
                    pt1.push_back(itkp->pt);

                    int xL = itkp->pt.x;
                    int yL = itkp->pt.y;

                    // 寻找边界
                    if(yL>max_u)
                        max_u=yL;
                    if(yL<min_u)
                        min_u=yL;
                    if(xL>max_v)
                        max_v=xL;
                    if(xL<min_v)
                        min_v=xL;
                }

                vector<uchar> status;
                vector<float> error;
                cv::calcOpticalFlowPyrLK(imLeft, imRight, pt1, pt2, status, error);

                cout << "pt1:" << pt1.size() << " pt2:" << pt2.size() << endl;

                /// 过滤
                vector<float> vtmp;
                for (int i = 0; i < pt2.size(); i++) {
                    if (status[i]) {
                        vtmp.push_back(pt1[i].x-pt2[i].x);
                        if(abs(pt1[i].y-pt2[i].y) >= 1)
                            status[i]=false;
                    }
                }

                sort(vtmp.begin(), vtmp.end(), [](float v1, float v2){return v1<v2;});
                float meida=vtmp[vtmp.size()/2];
                float minM=meida*0.8;
                float maxM=meida*1.2;

                for (int i = 0; i < pt2.size(); i++) {
                    if (status[i]) {
                        float disparity=pt1[i].x-pt2[i].x;
                        if(disparity>minM && disparity<maxM && disparity>=minD && disparity<maxD){
                            /// 计算深度
                            if(disparity<=0)
                            {
                                disparity=0.01;
                            }
                            float d = mbf/disparity;
                            vDepth.push_back(d);
                        }
                        else
                            vDepth.push_back(-1.0);
                    }
                    else
                        vDepth.push_back(-1.0);
                }

                //cout << "vkp:" << vkp.size() << " vDepth:" << vDepth.size() << endl;
                mmDepthDynamic.insert(pair<int, std::vector<float>>(label, vDepth));
                std::vector<cv::Point2i> vBoundary;
                vBoundary.push_back(cv::Point2i(min_v, min_u));
                vBoundary.push_back(cv::Point2i(max_v, max_u));
                //mmBoundaryDynamic.insert(pair<int, std::vector<cv::Point2i>>(label, vBoundary));
                //cout << "max_u" << max_u << " min_u" << min_u << " max_v" << max_v << " min_v" << min_v<< endl;
                itL++;
            }
            //cout << "L:" << itL->first-26000 << "," << itL->second.size() << endl;
        }
    }
    else{
        /// 采用SAD
        /// 首先根据中间的点寻找一个对应点，然后以其为初值作偏差，这样可以减少计算
        // 无初始值参数
        const int r_ = 3; // 卷积核半径
        const int x_ = 70; // 视察搜索范围x
        const int y_ = 5; // 视察搜索范围y
        // 有初始值参数
        const int R_ = 2; // 卷积核半径
        const int X_ = 5; // 视察搜索范围x
        const int Y_ = 2; // 视察搜索范围y
        const int threshold_ = 50; //
        const int img_col = imLeft.cols;
        const int img_row = imLeft.rows;

        while(itL!=itend){
            if(itL->second.size()<minNum){
                itL = mmKeysDynamic.erase(itL);
                itend = mmKeysDynamic.end();
                if(itL==itend)
                    break;
            }
            else{
                int label = itL->first;
                std::vector<cv::KeyPoint> vkp = itL->second;
                bool best_init=false;
                int best_init_x=0;
                int best_init_y=0;
                std::vector<float> vDepth;
                //std::vector<cv::Point2i> vBoundary;
                int max_u=-1, max_v=-1;
                int min_u=INT_MAX, min_v=INT_MAX;

                const int w = 5;
                for(auto itkp=vkp.begin(),itkpend=vkp.end();  itkp!=itkpend; itkp++){

                    int xL = itkp->pt.x;
                    int yL = itkp->pt.y;
                    if(yL>max_u)
                        max_u=yL;
                    if(yL<min_u)
                        min_u=yL;
                    if(xL>max_v)
                        max_v=xL;
                    if(xL<min_v)
                        min_v=xL;

                    //cout << "IL2" << IL << endl;
                    int bestDist = INT_MAX;
                    int uR=0;
                    if(!best_init){ /// 没有较好的初始匹配
                        if(xL-r_<0 || yL-r_<0 || xL+r_>=img_col || yL+r_>=img_row){
                            vDepth.push_back(-1.0);
                            continue;
                        }

                        cv::Mat IL = imLeft.rowRange(yL-r_, yL+r_).colRange(xL-r_, xL+r_);
                        IL.convertTo(IL,CV_32F);
                        IL = IL - IL.at<float>(r_,r_) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);

                        for(int col=0; col<=x_; col++){
                            for(int row=-y_; row<=y_; row++){
                                int xR = xL-col;
                                int yR = yL-row;
                                if(xR-r_<0 || yR-r_<0 || xR+r_>=img_col || yR+r_>=img_row)
                                    break;

                                cv::Mat IR = imRight.rowRange(yR-r_, yR+r_).colRange(xR-r_, xR+r_);
                                IR.convertTo(IR,CV_32F);
                                IR = IR - IR.at<float>(r_,r_) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                                //cout << "IR2" << IR << endl;
                                float dist = cv::norm(IL,IR,cv::NORM_L1);
                                if(dist<bestDist)
                                {
                                    bestDist = dist;
                                    best_init_x=xL-xR;
                                    best_init_y=yL-yR;
                                    uR = xR;
                                }
                            }
                        }
                        //cout << "bestDist1:" << bestDist << endl;
                        if(bestDist<threshold_){
                            best_init=true;
                            /// 计算深度
                            //float bestuR = uR;
                            //float disparity = (uL-bestuR);
                            float disparity = (xL-uR);

                            if(disparity>=minD && disparity<maxD)
                            {
                                if(disparity<=0)
                                {
                                    disparity=0.01;
                                    xL = xL-0.01;
                                }
                                float d = mbf/disparity;
                                vDepth.push_back(d);
                                //mvDepth[iL]=mbf/disparity;
                                //mvuRight[iL] = bestuR;
                                //vDistIdx.push_back(pair<int,int>(bestDist,iL));
                                //cout << "depth:" << mbf/disparity << endl;
                            }
                        }
                        else{
                            vDepth.push_back(-1.0);
                        }
                    }
                    else{ ///有了较好的初始匹配
                        if(xL-R_<0 || yL-R_<0 || xL+R_>=img_col || yL+R_>=img_row){
                            vDepth.push_back(-1.0);
                            continue;
                        }
                        cv::Mat IL = imLeft.rowRange(yL-R_, yL+R_).colRange(xL-R_, xL+R_);
                        IL.convertTo(IL,CV_32F);
                        IL = IL - IL.at<float>(R_,R_) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);


                        for(int col=-X_; col<=X_; col++){
                            for(int row=-Y_; row<=Y_; row++){
                                int xR = xL-best_init_x-col;
                                int yR = yL-best_init_y-row;
                                if(xR-R_<0 || yR-R_<0 || xR+R_>=img_col || yR+R_>=img_row)
                                    break;

                                cv::Mat IR = imRight.rowRange(yR-R_, yR+R_).colRange(xR-R_, xR+R_);
                                IR.convertTo(IR,CV_32F);
                                IR = IR - IR.at<float>(R_,R_) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                                float dist = cv::norm(IL,IR,cv::NORM_L1);
                                if(dist<bestDist)
                                {
                                    bestDist = dist;
                                    uR = xR;
                                }
                            }
                        }
                        if(bestDist<threshold_){
                            //best_init=true;
                            /// 计算深度
                            //float bestuR = uR;
                            //float disparity = (uL-bestuR);
                            float disparity = (xL-uR);

                            if(disparity>=minD && disparity<maxD)
                            {
                                if(disparity<=0)
                                {
                                    disparity=0.01;
                                    xL = xL-0.01;
                                }
                                float d = mbf/disparity;
                                vDepth.push_back(d);
                                //mvDepth[iL]=mbf/disparity;
                                //mvuRight[iL] = bestuR;
                                //vDistIdx.push_back(pair<int,int>(bestDist,iL));
                                //cout << "depth:" << mbf/disparity << endl;
                                //cout << "mbf:" << mbf << endl;
                            }
                        }
                        else{
                            vDepth.push_back(-1.0);
                        }
                        //cout << "bestDist2:" << bestDist << endl;
                    }

                }
                //cout << "vkp:" << vkp.size() << " vDepth:" << vDepth.size() << endl;
                mmDepthDynamic.insert(pair<int, std::vector<float>>(label, vDepth));
                std::vector<cv::Point2i> vBoundary;
                vBoundary.push_back(cv::Point2i(min_v, min_u));
                vBoundary.push_back(cv::Point2i(max_v, max_u));
                //mmBoundaryDynamic.insert(pair<int, std::vector<cv::Point2i>>(label, vBoundary));
                //cout << "max_u" << max_u << " min_u" << min_u << " max_v" << max_v << " min_v" << min_v<< endl;
                itL++;
            }
            //cout << "L:" << itL->first-26000 << "," << itL->second.size() << endl;
        }
    }

    //cout << "fram id: " << mnId << " mmKeysDynamic:" << mmKeysDynamic.size()  << " mmDepthDynamic:" << mmDepthDynamic.size() << endl;
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereoDynamic(const int &label, const int &i)
{
    const std::vector<cv::KeyPoint> vkp = mmKeysDynamic[label];
    const std::vector<float> vd=mmDepthDynamic[label];
    const float z = vd[i];

    if(z>0)
    {
        const float u = vkp[i].pt.x;
        const float v = vkp[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
