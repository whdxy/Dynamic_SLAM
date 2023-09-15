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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{
//int FrameDrawer::nLabelMin=-1; /// new

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                int classid=vCurrentKeys[i].class_id;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    if(classid == 26 ||  classid == 27 || classid == 28 || classid == 29 || classid == 30 || classid == 31 ||  classid == 32 || classid == 33){
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    }
                    else{
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    }

                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    // test 20230829
    /*
    ostringstream buffer;
    buffer << "ImageSemantic_keypoints_" << mFrameId << ".png";
    string imgfile = buffer.str();
    string imgpath = "/home/whd/SLAM/Dynamic_SLAM/Test/result/pic/" + imgfile;
    cv::imwrite(imgpath, im);
     */


    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawFrameNew()
{
    cv::Mat im, imDynamic;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    //std::map<int, std::vector<cv::KeyPoint>> mCurrentKeysDynamic; /// new, KeyPointsDynamic in current frame
    std::map<int, std::map<int, std::vector<int>>> mCurrentPixelsDynamic;
    std::map<int, std::vector<float>> mCurrentDepthDynamic;
    std::map<int, std::vector<int>> mCurrentBoundaryDynamic;
    std::map<int, std::vector<int>> mCurrentPixelsDynamicN;

    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);
        mIm.copyTo(imDynamic);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            mCurrentPixelsDynamic=mmCurrentPixelsDynamic;
            mCurrentDepthDynamic=mmCurrentDepthDynamic;
            mCurrentBoundaryDynamic=mmCurrentBoundaryDynamic;
            mCurrentPixelsDynamicN=mmCurrentPixelsDynamicN;

            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            mCurrentPixelsDynamic=mmCurrentPixelsDynamic;
            mCurrentDepthDynamic=mmCurrentDepthDynamic;
            mCurrentBoundaryDynamic=mmCurrentBoundaryDynamic;
            mCurrentPixelsDynamicN=mmCurrentPixelsDynamicN;

            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            mCurrentPixelsDynamic=mmCurrentPixelsDynamic;
            mCurrentDepthDynamic=mmCurrentDepthDynamic;
            mCurrentBoundaryDynamic=mmCurrentBoundaryDynamic;
            mCurrentPixelsDynamicN=mmCurrentPixelsDynamicN;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);
    if(imDynamic.channels()<3) //this should be always true
        cvtColor(imDynamic,imDynamic,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                         cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                int classid=vCurrentKeys[i].class_id;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }

        /// -------------------
        auto itPixelsDynamic=mCurrentPixelsDynamic.begin(), itPixelsDynamicend=mCurrentPixelsDynamic.end();
        //auto itDepthDynamic=mCurrentFrame.mmDepthDynamic.begin();
        while(itPixelsDynamic!=itPixelsDynamicend){
            int label = itPixelsDynamic->first;
            std::map<int, std::vector<int>> m = itPixelsDynamic->second;
            std::vector<float> vd = mCurrentDepthDynamic[label];
            std::vector<int> vN = mCurrentPixelsDynamicN[label];

            auto itm=m.begin(), itmend=m.end();
            int rowInit=itm->first;

            cv::Scalar scalar;
            ChooseColor(label, scalar);

            int num=0;
            for(; itm!=itmend; itm++){
                //cout << "   itPixelsDynamic:" << itm->second.size() << " vd:" << vd.size() << " vN:" << vN[num++] << endl;
                int row=itm->first;
                std::vector<int> vcol=itm->second;
                for(int i=0; i<vcol.size(); i++){
                    float d=vd[vN[num]+i];
                    if(d>0){
                        cv::Point2f pt;
                        pt.x=vcol[i];
                        pt.y=row;
                        cout << "vcol[i]:" << vcol[i] << " row:" << row << endl;
                        cout << "pt.x:" << pt.x << " pt.y:" << pt.y << endl;
                        //cv::circle(im,pt,3,scalar,-1);
                    }
                }

                //cout << endl;
                num++;
            }

            itPixelsDynamic++;
        }

        /// ==============================
        /*
        auto it=mCurrentPixelsDynamic.begin(), itend=mCurrentPixelsDynamic.end();
        //auto itdepth=mCurrentDepthDynamic.begin(), itenddepth=mCurrentDepthDynamic.end();
        for( ; it!=itend; it++)
        {
            int label = it->first;
            std::map<int, std::vector<int>> m = it->second;
            std::vector<float> vd = mCurrentDepthDynamic[label];
            std::vector<int> vN = mCurrentPixelsDynamicN[label];

            cv::Scalar scalar;
            ChooseColor(label, scalar);

            auto itm=m.begin(), itmend=m.end();
            //int rowInit=itm->first;
            int colN=0;
            for(;itm!=itmend;itm++){
                int row=itm->first;
                //cout << "row:" << row;
                std::vector<int> vcol=itm->second;
                for(int i=0; i<vcol.size(); i++){

                    //cout << " col:" << i ;
                    float d=vd[vN[colN]+i];
                    if(d>0){
                        cv::Point2f pt;
                        pt.x=vcol[i];
                        pt.y=row;
                        cv::circle(im,pt,3,scalar,-1);
                        //cout << " | d:" << d;
                    }

                }
                colN++;
                //cout << endl;
            }

            //cout << "label: " << label << "  num:" << vkp.size() << endl;
        } */
    }
    //cout << "fram id: " << mFrameId << "  ---------" << endl;

    // test 20230829 保存图片
    /*
    ostringstream buffer;
    buffer << "ImageSemantic_keypoints_" << mFrameId << ".png";
    string imgfile = buffer.str();
    string imgpath = "/home/whd/SLAM/Dynamic_SLAM/Test/result/pic/" + imgfile;
    cv::imwrite(imgpath, im);
    */
    /// 上面是show特帧点和像素点
    /// -------------------
    /// 下面show动态物体速度

    for(auto itBoundary=mCurrentBoundaryDynamic.begin(), itBoundaryend=mCurrentBoundaryDynamic.end() ; itBoundary!=itBoundaryend; itBoundary++)
    {
        cv::Point2i p1,p2;

        p1.x=itBoundary->second[0];
        p1.y=itBoundary->second[2];
        p2.x=itBoundary->second[1];
        p2.y=itBoundary->second[3];

        cv::rectangle(imDynamic, p1, p2, cv::Scalar(0,255,0), 2);
    }



    cv::Mat im_combine(im.rows*2, im.cols, im.type());
    //cout << "im: " << im.size() << " " << im.type() << " im_combine: " << im_combine.size() << " " << im_combine.type() << endl;
    im.copyTo(im_combine.rowRange(0,im.rows).colRange(0,im.cols));
    imDynamic.copyTo(im_combine.rowRange(im.rows,im.rows*2).colRange(0,im.cols));
    cv::Mat imWithInfo;
    DrawTextInfo(im_combine,state, imWithInfo);

    return imWithInfo;
}

/// new add
void FrameDrawer::ChooseColor(int label, cv::Scalar &scalar){
    label = label - Frame::nLabelMin;
    //cout << "label:" << label << endl;
    while(label>15)
        label=label-15;

    switch(label)
    {
        case 0:
            scalar = cv::Scalar(250,51,153); //BGR
            break;
        case 1:
            scalar = cv::Scalar(214,112,218);
            break;
        case 2:
            scalar = cv::Scalar(20,97,199);
            break;
        case 3:
            scalar = cv::Scalar(45,82,160);
            break;
        case 4:
            scalar = cv::Scalar(143,143,188);
            break;
        case 5:
            scalar = cv::Scalar(95,164,244);
            break;
        case 6:
            scalar = cv::Scalar(42,42,128);
            break;
        case 7:
            scalar = cv::Scalar(0,128,255);
            break;
        case 8:
            scalar = cv::Scalar(0,215,255);
            break;
        case 9:
            scalar = cv::Scalar(205,245,255);
            break;
        case 10:
            scalar = cv::Scalar(192,192,192);
            break;
        case 11:
            scalar = cv::Scalar(140,199,0);
            break;
        case 12:
            scalar = cv::Scalar(122,25,25);
            break;
        case 13:
            scalar = cv::Scalar(158,168,3);
            break;
        case 14:
            scalar = cv::Scalar(34,139,34);
            break;
        case 15:
            scalar = cv::Scalar(0,255,0);
            break;
    }

    /*
    while(label>50)
        label=label-50;

    switch(label)
    {
        case 0:
            scalar = cv::Scalar(255,255,240); //lvory
            break;
        case 1:
            scalar = cv::Scalar(0,0,255); //blue
            break;
        case 2:
            scalar = cv::Scalar(255,0,0); //red
            break;
        case 3:
            scalar = cv::Scalar(255,255,0); // Yellow1
            break;
        case 4:
            scalar = cv::Scalar(47,255,173);
            break;
        case 5:
            scalar = cv::Scalar(128, 0, 128);
            break;
        case 6:
            scalar = cv::Scalar(203,192,255);
            break;
        case 7:
            scalar = cv::Scalar(196,228,255);
            break;
        case 8:
            scalar = cv::Scalar(42,42,165);
            break;
        case 9:
            scalar = cv::Scalar(255,255,255);
            break;
        case 10:
            scalar = cv::Scalar(139,101,8); // 245,245,245
            break;
        case 11:
            scalar = cv::Scalar(0,165,255);
            break;
        case 12:
            scalar = cv::Scalar(230,216,173);
            break;
        case 13:
            scalar = cv::Scalar(128,128,128);
            break;
        case 14:
            scalar = cv::Scalar(0,215,255);
            break;
        case 15:
            scalar = cv::Scalar(30,105,210);
            break;
        case 16:
            scalar = cv::Scalar(0,255,0);
            break;
        case 17:
            scalar = cv::Scalar(34, 34, 178);
            break;
        case 18:
            scalar = cv::Scalar(240, 255, 240);
            break;
        case 19:
            scalar = cv::Scalar(250, 206, 135);
            break;
        case 20:
            scalar = cv::Scalar(238, 104, 123);
            break;
        case 21:
            scalar = cv::Scalar(225, 228, 255);
            break;
        case 22:
            scalar = cv::Scalar(128, 0, 0);
            break;
        case 23:
            scalar = cv::Scalar(35, 142, 107);
            break;
        case 24:
            scalar = cv::Scalar(45, 82, 160);
            break;
        case 25:
            scalar = cv::Scalar(0, 255, 127);
            break;
        case 26:
            scalar = cv::Scalar(139, 0, 0);
            break;
        case 27:
            scalar = cv::Scalar(60, 20, 220);
            break;
        case 28:
            scalar = cv::Scalar(0, 0, 139);
            break;
        case 29:
            scalar = cv::Scalar(211, 0, 148);
            break;
        case 30:
            scalar = cv::Scalar(255, 144, 30);
            break;
        case 31:
            scalar = cv::Scalar(105, 105, 105);
            break;
        case 32:
            scalar = cv::Scalar(180, 105, 255);
            break;
        case 33:
            scalar = cv::Scalar(204, 209, 72);
            break;
        case 34:
            scalar = cv::Scalar(173, 222, 255);
            break;
        case 35:
            scalar = cv::Scalar(143, 143, 188);
            break;
        case 36:
            scalar = cv::Scalar(50, 205, 50);
            break;
        case 37:
            scalar = cv::Scalar(34, 34, 178);
            break;
        case 38:
            scalar = cv::Scalar(240, 255, 240);
            break;
        case 39:
            scalar = cv::Scalar(250, 206, 135);
            break;
        case 40:
            scalar = cv::Scalar(238, 104, 123);
            break;
        case 41:
            scalar = cv::Scalar(225, 228, 255);
            break;
        case 42:
            scalar = cv::Scalar(128, 0, 0);
            break;
        case 43:
            scalar = cv::Scalar(35, 142, 107);
            break;
        case 44:
            scalar = cv::Scalar(45, 82, 160);
            break;
        case 45:
            scalar = cv::Scalar(30,105,210);
            break;
        case 46:
            scalar = cv::Scalar(32,178,170);
            break;
        case 47:
            scalar = cv::Scalar(34, 34, 178);
            break;
        case 48:
            scalar = cv::Scalar(240, 255, 240);
            break;
        case 49:
            scalar = cv::Scalar(250, 206, 135);
            break;
        case 50:
            scalar = cv::Scalar(238, 104, 123);
            break;
    }
     */
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "Fs: " << mFrameId << ", KFs: " << nKFs << ", FPS: " << mFPS << ", MPs: " << nMPs << ", Matches: " << mnTracked ;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+18,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+18,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),1.5,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);

    //if(nLabelMin==-1)
    //    nLabelMin=pTracker->mCurrentFrame.nLabelMin;

    pTracker->mImGray.copyTo(mIm);
    //pTracker->mImGraySemantic.copyTo(mIm); // test 20230901
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mmCurrentPixelsDynamic=pTracker->mCurrentFrame.mmPixelsDynamic;
    mmCurrentDepthDynamic=pTracker->mCurrentFrame.mmDepthDynamic;
    mmCurrentBoundaryDynamic=pTracker->mCurrentFrame.mmBoundaryDynamic;
    mmCurrentPixelsDynamicN=pTracker->mCurrentFrame.mmPixelsDynamicN;

    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    mFPS = pTracker->ComputeFPS();
    mFrameId = pTracker->mCurrentFrame.mnId;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
