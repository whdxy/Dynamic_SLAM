//
// Created by whd on 23-10-16.
//

#include "InstanceTracking.h"

namespace ORB_SLAM2 {

    InstanceTracking::InstanceTracking(){}

    InstanceTracking::InstanceTracking(float bf, float fx, float fy, float cx, float cy)
    : mbf(bf), mfx(fx), mfy(fy), mcx(cx), mcy(cy) {

        mH = 1.5;

        mK=cv::Mat::zeros(3, 3, CV_64FC1);

        mK.at<double>(0, 0) = mfx;
        mK.at<double>(1, 1) = mfy;
        mK.at<double>(0, 2) = mcx;
        mK.at<double>(1, 2) = mcy;
        mK.at<double>(2, 2) = 1.0;

    }

    void InstanceTracking::FastInitialization(int threshold) {
        FAST = cv::FastFeatureDetector::create(threshold);
    }

    // old: 返回未分类的fast角点
    void InstanceTracking::FastDetect(const cv::Mat &img, const cv::Mat &imgSem,
                                      const std::map<int, std::vector<cv::Point2i>> &boundary,
                                      std::vector<cv::Point2f> &pt) {
        std::vector<cv::KeyPoint> kp, kpSem;
        FAST->detect(img, kp);

        for (auto &KP: kp) {
            int label = imgSem.at<char16_t>(KP.pt.y, KP.pt.x);
            if (label > 26000 && label < 27000) {
                auto it = boundary.find(label);
                if (it != boundary.end()) {
                    cv::Point2i p1, p2;
                    p1 = it->second[0];
                    p2 = it->second[1];
                    //std::cout << p1 << std::endl;
                    //std::cout << p2 << std::endl;
                    if (p1.x < KP.pt.x && p2.x > KP.pt.x && p1.y < KP.pt.y && p2.y > KP.pt.y) {
                        kpSem.push_back(KP);
                        pt.push_back(KP.pt);
                    }
                }

            }
        }
        /*
        cv::Mat img_;
        img.copyTo(img_);
        cv::drawKeypoints(img, kpSem, img_);
        cv::imshow("img",img_);
        //cv::imshow("imgSem",imgSem);
        cv::waitKey(0);
         */

    }

    // new: 返回分类的fast角点，用于初始化
    void InstanceTracking::FastDetect(const cv::Mat &img, const cv::Mat &imgSem,
                                      const std::map<int, std::vector<cv::Point2i>> &boundary,
                                      std::map<int, std::vector<cv::Point2f>> &pt) {
        std::vector<cv::KeyPoint> kp;
        FAST->detect(img, kp);
        for (auto &KP: kp) {
            int label = imgSem.at<char16_t>(KP.pt.y, KP.pt.x);
            if (label > 26000 && label < 27000) {
                auto it = boundary.find(label);
                if (it != boundary.end()) {
                    cv::Point2i p1, p2;
                    p1 = it->second[0];
                    p2 = it->second[1];

                    if (p1.x < KP.pt.x && p2.x > KP.pt.x && p1.y < KP.pt.y && p2.y > KP.pt.y) {
                        //pt.push_back(KP.pt);
                        auto itP = pt.find(label);
                        if (itP != pt.end())
                            itP->second.push_back(KP.pt);
                        else
                            pt.insert(std::pair<int, std::vector<cv::Point2f>>(label, {KP.pt}));
                    }
                }

            }
        }
    }

    // old: 返回分类的fast角点，用于追踪(补充，在追踪的基础上+新提取到的)
    void InstanceTracking::FastDetect(const cv::Mat &img, const cv::Mat &imgSem,
                                      const std::map<int, std::vector<cv::Point2i>> &boundary,
                                      std::map<int, std::vector<cv::Point2f>> &ptCur,
                                      const std::map<int, std::vector<cv::Point2f>> &ptLast) {
        std::vector<cv::KeyPoint> kp;
        FAST->detect(img, kp);
        for (auto &KP: kp) {
            int label = imgSem.at<char16_t>(KP.pt.y, KP.pt.x);
            if (label > 26000 && label < 27000) {
                auto it = boundary.find(label);
                if (it != boundary.end()) {
                    cv::Point2i p1, p2;
                    p1 = it->second[0];
                    p2 = it->second[1];

                    if (p1.x < KP.pt.x && p2.x > KP.pt.x && p1.y < KP.pt.y && p2.y > KP.pt.y) {
                        //pt.push_back(KP.pt);
                        auto itP = ptCur.find(label);
                        if (itP != ptCur.end())
                            itP->second.push_back(KP.pt);
                        else
                            ptCur.insert(std::pair<int, std::vector<cv::Point2f>>(label, {KP.pt}));
                    }
                }
            }
        }

        /*
        // 可视化
        cv::Mat img2_CV;
        cv::cvtColor(img, img2_CV, cv::COLOR_GRAY2BGR);

        // 画当前帧fast提取
        for (auto it = ptCur.begin(), itend = ptCur.end(); it != itend; it++) {
            std::vector<cv::Point2f> vpt = it->second;
            for (int i = 0; i < vpt.size(); i++) {
                cv::circle(img2_CV, vpt[i], 2, cv::Scalar(0, 250, 0), 1.5);
            }
        }

        /// 比较当前帧fast提取与前一帧光流追踪的角点差
        for (auto it = ptLast.begin(), itend = ptLast.end(); it != itend; it++) {

            int label = it->first;
            auto itCur = ptCur.find(label);
            if (itCur != ptCur.end()) {
                std::vector<cv::Point2f> ptC = itCur->second;
                std::vector<cv::Point2f> ptL = it->second;
                std::sort(ptC.begin(), ptC.end(), [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.x < pt2.x; });
                std::sort(ptC.begin(), ptC.end(), [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.y < pt2.y; });
                std::sort(ptL.begin(), ptL.end(), [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.x < pt2.x; });
                std::sort(ptL.begin(), ptL.end(), [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.y < pt2.y; });

                std::vector<cv::Point2f> diff;

                // diff保存Last中有，但Cur没有点角点
                std::set_difference(ptL.begin(), ptL.end(), ptC.begin(), ptC.end(), std::back_inserter(diff),
                                    [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.x == pt2.x && pt1.y == pt2.y; });

                //std::cout << "label: " << label << std::endl;
                //for(auto pt: diff){
                //    std::cout << "  " << pt << std::endl;
                //}

                // 画Last中有，但Cur没有点角点
                for (int i = 0; i < diff.size(); i++) {
                    cv::circle(img2_CV, diff[i], 2, cv::Scalar(0, 0, 255), 2);
                }
            }
        }
         */

        //cv::imshow("tracked by opencv", img2_CV);
        //cv::imwrite("/home/whd/SLAM/pic20231017/optical1.png", img2_CV);
        //cv::waitKey(0);
    }

    struct PointCompare{
        bool operator()(const cv::Point2f p1, const cv::Point2f p2) const{
            if(p1.x != p2.x)
                return p1.x < p2.x;
            if(p1.y != p2.y)
                return p1.y < p2.y;
        }
    };

    // new: 返回分类的fast角点，用于追踪(补充，在追踪的基础上+新提取到的)
    void InstanceTracking::FastDetectFill(const cv::Mat &img, const cv::Mat &imgSem,
                                      const std::map<int, std::vector<cv::Point2i>> &boundary,
                                      std::map<int, std::vector<cv::Point2f>> &pt) {

        //std::cout << "boundary：" << boundary.size() << std::endl;

        /// step1、提取特征点
        std::vector<cv::KeyPoint> kp;
        FAST->detect(img, kp);

        /// step2、对新提取的特征点进行过滤
        /// 1、车辆上特征点 2、边界之内
        std::map<int, std::vector<cv::Point2f>> ptNew;
        for (auto &KP: kp) {
            int label = imgSem.at<char16_t>(KP.pt.y, KP.pt.x);
            if (label > 26000 && label < 27000) {
                //std::cout << "  label：" << label << std::endl;
                auto it = boundary.find(label);
                if (it != boundary.end()) {
                    cv::Point2i p1, p2;
                    p1 = it->second[0];
                    p2 = it->second[1];

                    if (p1.x < KP.pt.x && p2.x > KP.pt.x && p1.y < KP.pt.y && p2.y > KP.pt.y) {
                        //pt.push_back(KP.pt);
                        auto itP = ptNew.find(label);
                        if (itP != ptNew.end())
                            itP->second.push_back(KP.pt);
                        else
                            ptNew.insert(std::pair<int, std::vector<cv::Point2f>>(label, {KP.pt}));
                    }
                }
            }
        }

        /// step3、将刚刚检测出的特征点加入到光流追踪到的特征点中（比较当前帧fast提取与前一帧光流追踪的角点差）
        //std::cout << "*****************************" << std::endl;
        for (auto it = ptNew.begin(), itend = ptNew.end(); it != itend; it++){
            int label = it->first;
            auto itOld = pt.find(label);
            if (itOld != pt.end()){
                std::set<cv::Point2f, PointCompare> spt;
                spt.insert(itOld->second.begin(), itOld->second.end());
                //std::cout << "spt" << spt.size() << std::endl;
                for(auto itPoint=it->second.begin(),itPointend=it->second.end(); itPoint!=itPointend; itPoint++){
                    auto itPointOld = spt.find(*itPoint);
                    if(itPointOld==spt.end()){
                        //std::cout << "123" << std::endl;

                    }

                }
            }
            else{
                std::vector<cv::Point2f> vpTemp = it->second;
                pt.insert(std::pair<int, std::vector<cv::Point2f>>(label, vpTemp));
                //itOld->second.insert(itOld->second.end(), it->second.begin(),it->second.end());
                //std::cout << "2222" << std::endl;
            }
        }

        /*
        for (auto it = pt.begin(), itend = pt.end(); it != itend; it++) {

            int label = it->first;
            auto itNew = ptNew.find(label);
            if (itNew != ptNew.end()) {
                std::vector<cv::Point2f> ptC = itNew->second;
                std::vector<cv::Point2f> ptL = it->second;
                std::sort(ptC.begin(), ptC.end(), [](cv::Point2f pt1, cv::Point2f pt2)
                { if(pt1.x != pt2.x) return pt1.x < pt2.x; if(pt1.y != pt2.y) return pt1.y < pt2.y;});
                std::sort(ptL.begin(), ptL.end(), [](cv::Point2f pt1, cv::Point2f pt2)
                { if(pt1.x != pt2.x) return pt1.x < pt2.x; if(pt1.y != pt2.y) return pt1.y < pt2.y;});

                std::vector<cv::Point2f> diff;

                // diff保存Last中有，但Cur没有点角点
                //std::set_difference(ptL.begin(), ptL.end(), ptC.begin(), ptC.end(), std::back_inserter(diff),
                //                    [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.x == pt2.x && pt1.y == pt2.y; });

                std::set_difference(ptC.begin(), ptC.end(), ptL.begin(), ptL.end(), std::back_inserter(diff),
                                    [](cv::Point2f pt1, cv::Point2f pt2) { return pt1.x == pt2.x && pt1.y == pt2.y; });

                //itNew->second.insert(itNew->second.end(), diff.begin(), diff.end());
            }
        }
         */

        // 用set融合特帧点
        /*
        std::map<int, std::set<cv::Point2f, PointCompare>> mpt;
        for(auto it=pt.begin(), itend=pt.end(); it!=itend; it++){

            //std::set<cv::Point2f, PointCompare> sptTmp = std::set<cv::Point2f, PointCompare>(it->second.begin(), it->second.end());
            mpt.insert(std::pair<int,std::set<cv::Point2f, PointCompare>>(it->first, std::set<cv::Point2f, PointCompare>(it->second.begin(), it->second.end())));
        }
        for(auto it=ptNew.begin(), itend=ptNew.end(); it!=itend; it++){
            auto it_ = pt.find(it->first);
            if(it_ != pt.end()){
                mpt[it->first].insert(it->second.begin(), it->second.end());
            }

        }

        //
        for(auto it=mpt.begin(), itend=mpt.end(); it!=itend; it++){
            int label = it->first;
            std::set<cv::Point2f, PointCompare> sTemp=it->second;
            auto itpt=pt.find(label);
            if(itpt!=pt.end())
                it
        }

        std::vector<cv::Point2f> vptTemp=std::vector<cv::Point2f>(spt.begin(),spt.end());
        pt.swap(vptTemp);
         */

    }


    // 1017证明：每个图像分开光流与整体光流效果一样
    // OptFlowID 当前在特帧点与上一帧点对应id
    void InstanceTracking::OpticalFlowPyrLK(const cv::Mat &imgCur, const cv::Mat &imgLast, const cv::Mat &imgSem,
                                            std::map<int, std::vector<cv::Point2f>> &ptCur,  std::map<int, std::vector<int>> &OptFlowID,
                                            const std::map<int, std::vector<cv::Point2f>> &ptLast) {

        std::map<int, std::vector<cv::Point2f>> ptCurTemp;
        std::map<int, std::vector<int>> OptFlowIDTemp;
        // 可视化参数
        //std::vector<cv::Point2f> pt_show_cur, pt_show_last;

        //std::cout << "========================" << std::endl;
        for (auto it = ptLast.begin(), itend = ptLast.end(); it != itend; it++) {

            int label=it->first;
            std::vector<cv::Point2f> vp=it->second;
            if(vp.size() < 20)
                continue;

            ptCurTemp.insert(std::pair<int, std::vector<cv::Point2f>>(label, {}));
            OptFlowIDTemp.insert(std::pair<int, std::vector<int>>(label, {}));

            std::vector<uchar> status;
            std::vector<float> error;
            std::vector<cv::Point2f> pt_cur;
            //std::cout << "label:" << it->first << " " << it->second.size() << std::endl;
            cv::calcOpticalFlowPyrLK(imgLast, imgCur, vp, pt_cur, status, error);
            for (int i = 0; i < status.size(); i++) {
                if (status[i]) {
                    if (imgSem.at<char16_t>(pt_cur[i].y, pt_cur[i].x) == label) {
                        ptCurTemp[label].push_back(pt_cur[i]);
                        OptFlowIDTemp[label].push_back(i);
                        //pt_show_cur.push_back(pt_cur[i]);
                        //pt_show_last.push_back(vp[i]);
                    }
                }
            }
        }

        /// 删除fast角点小于5个的label
        for (auto it = ptCurTemp.begin(), itend = ptCurTemp.end(); it != itend; it++) {
            if (it->second.size() > 5){
                ptCur.insert(std::pair<int, std::vector<cv::Point2f>>(it->first, it->second));
                OptFlowID.insert(std::pair<int, std::vector<int>>(it->first, OptFlowIDTemp[it->first]));
            }
        }

        /// 可视化
        /*
        cv::Mat img2_CV;
        cv::cvtColor(imgCur, img2_CV, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < pt_show_cur.size(); i++) {
            cv::circle(img2_CV, pt_show_cur[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt_show_last[i], pt_show_cur[i], cv::Scalar(0, 250, 0));
        }
         */

        //cv::imshow("tracked by opencv", img2_CV);
        //cv::imwrite("/home/whd/SLAM/pic20231017/optical1.png", img2_CV);
        //cv::waitKey(200);
    }

    void InstanceTracking::StereoMatchingInitialization(int Channels, int PreFilterCap, int UniquenessRatio,
                                                        int SpeckleRange) {
        mChannels = Channels;
        mPreFilterCap = PreFilterCap;
        mUniquenessRatio = UniquenessRatio;
        mSpeckleRange = SpeckleRange;
        SGBM = cv::StereoSGBM::create();
        SGBM->setPreFilterCap(mPreFilterCap);
        SGBM->setUniquenessRatio(mUniquenessRatio);
        SGBM->setSpeckleRange(mSpeckleRange);
        SGBM->setMode(cv::StereoSGBM::MODE_HH);
    }


    // old（整体匹配，时间消耗大）
    void InstanceTracking::ComputeDisp(const cv::Mat &imgL, const cv::Mat &imgR,
                                       const std::map<int, std::vector<cv::Point2f>> &pt,
                                       std::map<int, std::vector<float>> &depth,
                                       int SADWindowSize, int MinDisparity, int NumDisparities) {

        SGBM->setBlockSize(SADWindowSize);
        int p1 = 8 * mChannels * SADWindowSize * SADWindowSize;
        int p2 = 32 * mChannels * SADWindowSize * SADWindowSize;
        SGBM->setP1(p1);
        SGBM->setP2(p2);

        SGBM->setMinDisparity(MinDisparity);
        SGBM->setNumDisparities(NumDisparities);

        cv::Mat temp;
        SGBM->compute(imgL, imgR, temp);
        //std::cout << "type: " << temp.type() << std::endl;
        temp.convertTo(temp, CV_32F, 1.0 / 16);
        //std::cout << "type: " << temp.type() << std::endl;

        ComputeDepth(temp, pt, depth);

        // show
        //cv::Mat imgDisp = cv::Mat::zeros(temp.size(), CV_8UC1);
        //cv::normalize(temp,imgDisp, 0, 255 ,cv::NORM_MINMAX);
        //cv::convertScaleAbs(imgDisp,imgDisp);

        //cv::imshow("imgDisp", imgDisp);
        //cv::imwrite("/home/whd/SLAM/pic20231017/imgDisp.png", imgDisp);
        //cv::waitKey(0);
    }

    // new（实例区域匹配）
    void InstanceTracking::ComputeDisp(const cv::Mat &imgL, const cv::Mat &imgR, const std::map<int, std::vector<cv::Point2i>> &boundary,
                     const std::map<int, std::vector<cv::Point2f>> &pt, std::map<int, std::vector<float>> &depth,
                     int SADWindowSize, int MinDisparity, int NumDisparities){

        //std::cout << "boundary: " << boundary.size() << std::endl;
        //std::cout << "pt: " << pt.size() << std::endl;

        SGBM->setBlockSize(SADWindowSize);
        int p1 = 8 * mChannels * SADWindowSize * SADWindowSize;
        int p2 = 32 * mChannels * SADWindowSize * SADWindowSize;
        SGBM->setP1(p1);
        SGBM->setP2(p2);

        SGBM->setMinDisparity(MinDisparity);

        //const int rate = 3;

        for(auto itBoundary=boundary.begin(), itBoundaryend=boundary.end(); itBoundary != itBoundaryend; itBoundary++){
            int label = itBoundary->first;

            auto itpt = pt.find(label);
            if(itpt!=pt.end()){
                cv::Point2i p1, p2;
                p1=itBoundary->second[0];
                p2=itBoundary->second[1];

                int width=p2.x-p1.x;
                int heigth=p2.y-p1.y;

                // 计算视察，根据视差，设置SGBM的NumDisparities
                float d = mH*mfy/heigth;
                float disp = mbf*heigth/(mH*mfy);

                int numDisparities = 16;
                if(disp<16)
                    numDisparities = 16;
                else if(disp<32)
                    numDisparities = 32;
                else if(disp<64)
                    numDisparities = 64;
                else if(disp<128)
                    numDisparities = 128;


                //std::cout << "label: " << label <<std::endl;

                //std::cout << "boundary: " << p1 << "  " << p2 <<std::endl;

                if(p1.x-numDisparities<0)
                    width = p2.x;
                else
                    width += numDisparities;

                cv::Rect roi(p2.x-width,p1.y,width,heigth);
                cv::Mat imgL_Rect = imgL(roi);
                cv::Mat imgR_Rect = imgR(roi);

                /*
                if(heigth>160)
                    numDisparities *= 8;
                else if(heigth>100)
                    numDisparities *= 4;
                else if(heigth>100)
                    numDisparities *= 2;
                */

                SGBM->setNumDisparities(numDisparities);

                cv::Mat temp;
                SGBM->compute(imgL_Rect, imgR_Rect, temp);
                //std::cout << "type: " << temp.type() << std::endl;
                temp.convertTo(temp, CV_32F, 1.0 / 16);

                std::cout << "label: " << label << " width: " << width << " heigth: " << heigth << " d: " << d <<
                " disp: " << disp << " numDisparities: " << numDisparities << std::endl;

                cv::Point2f pt_l_u;
                pt_l_u.x=p2.x-width;
                pt_l_u.y=p1.y;

                std::vector<cv::Point2f> vpt=itpt->second;
                std::vector<float> vdepth;

                ComputeDepth(temp, vpt, pt_l_u, depth[label]);

                cv::Mat img_combine(imgL_Rect.rows*3, imgL_Rect.cols, imgL_Rect.type());
                imgL_Rect.copyTo(img_combine.rowRange(0,imgL_Rect.rows).colRange(0,imgL_Rect.cols));
                imgR_Rect.copyTo(img_combine.rowRange(imgL_Rect.rows,imgL_Rect.rows*2).colRange(0,imgL_Rect.cols));

                cv::Mat imgDisp = cv::Mat::zeros(temp.size(), CV_8UC1);
                cv::normalize(temp,imgDisp, 0, 255 ,cv::NORM_MINMAX);
                cv::convertScaleAbs(imgDisp,imgDisp);
                imgDisp.copyTo(img_combine.rowRange(imgL_Rect.rows*2,imgL_Rect.rows*3).colRange(0,imgL_Rect.cols));

                //std::string s_ = "/home/whd/SLAM/pic20231017/" + std::to_string(label) + "_disp_" + std::to_string(disp) + ".png";
                //cv::imwrite(s_, img_combine);

                //cv::imshow(s_,img_combine);
                //std::string s1 = "/home/whd/SLAM/pic20231017/" + std::to_string(label) + "_left.png";
                //std::string s2 = "/home/whd/SLAM/pic20231017/" + std::to_string(label) + "_right.png";
                //cv::imshow("temp",temp);
                //cv::imshow(s1,imgL_Rect);
                //cv::imshow(s2,imgR_Rect);
                //cv::imwrite(s1, imgL_Rect);
                //cv::imwrite(s2, imgR_Rect);
                //cv::waitKey(0);

            }
        }


        /*
        SGBM->setNumDisparities(NumDisparities);

        cv::Mat temp;
        SGBM->compute(imgL, imgR, temp);
        //std::cout << "type: " << temp.type() << std::endl;
        temp.convertTo(temp, CV_32F, 1.0 / 16);
        //std::cout << "type: " << temp.type() << std::endl;

        ComputeDepth(temp, pt, depth);
         */

        // show
        //cv::Mat imgDisp = cv::Mat::zeros(temp.size(), CV_8UC1);
        //cv::normalize(temp,imgDisp, 0, 255 ,cv::NORM_MINMAX);
        //cv::convertScaleAbs(imgDisp,imgDisp);

        //cv::imshow("imgDisp", imgDisp);
        //cv::imwrite("/home/whd/SLAM/pic20231017/imgDisp.png", imgDisp);
        //cv::waitKey(0);
    }

    // old
    void InstanceTracking::ComputeDepth(const cv::Mat &imgDisp, const std::map<int, std::vector<cv::Point2f>> &pt,
                                        std::map<int, std::vector<float>> &depth) {
        for (auto it = pt.begin(), itend = pt.end(); it != itend; it++) {

            bool b = false;
            // 计算深度
            //std::cout << "label:" << it->first << std::endl;
            std::vector<float> vf(it->second.size(), -1.0);
            depth.insert(std::pair<int, std::vector<float>>(it->first, vf));
            for (int i = 0; i < it->second.size(); i++) {
                cv::Point2f p = it->second[i];
                float disp = imgDisp.at<float>(p.y, p.x);
                if (disp > 0) {
                    float d = mbf / disp;
                    if (d < 30) {
                        depth[it->first][i] = d;
                        //std::cout << " | " << d;

                        if (!b)
                            b = true;
                    }
                }

            }

            // 删除离群点
            float stdev = 3.0; // 标准差
            while (stdev > 2.5 && b) {
                int sum = 0, num = 0;
                stdev = 0.0;

                // 计算平均数
                for (int i = 0; i < it->second.size(); i++) {
                    if (depth[it->first][i] > 0) {
                        //std::cout << depth[it->first][i] << std::endl;
                        sum += depth[it->first][i];
                        num++;
                    }
                }

                // 计算标准差
                float mean = (float) sum / num;
                for (int i = 0; i < it->second.size(); i++) {
                    if (depth[it->first][i] > 0) {
                        stdev += (depth[it->first][i] - mean) * (depth[it->first][i] - mean);
                    }
                }
                stdev = sqrt(stdev / num);

                for (int i = 0; i < it->second.size(); i++) {
                    if (depth[it->first][i] > 0) {
                        if (abs(depth[it->first][i] - mean) > stdev)
                            depth[it->first][i] = -1.0;
                    }
                }
                //std::cout << std::endl << "sum:" << sum << "  num:" << num << "  mean:" << mean << "  stdev:" << stdev << std::endl;
            }
            //std::cout << std::endl << std::endl;
        }
    }

    // new
    void InstanceTracking::ComputeDepth(const cv::Mat &imgDisp, const std::vector<cv::Point2f> &pt, const cv::Point2f &pt_left_upper, std::vector<float> &depth){
        bool b = false;

        // 计算深度
        //std::vector<float> vf(pt.size(), -1.0);
        depth = std::vector<float>(pt.size(), -1.0);
        for (int i = 0; i < pt.size(); i++) {
            cv::Point2f p = pt[i];
            //std:: cout << "imgDisp, " << imgDisp.cols << ", " <<  imgDisp.rows << std::endl
            //        << "y, " << p.y << "  x, " <<  p.x << std::endl
            //       << "pt_left_uppery, " << pt_left_upper.y << "  pt_left_upperx," << pt_left_upper.x << std::endl;
            float disp = imgDisp.at<float>(p.y-pt_left_upper.y, p.x-pt_left_upper.x);
            if (disp > 0) {
                float d = mbf / disp;
                if (d < 30) {
                    depth[i] = d;
                    if (!b)
                        b = true;
                }
            }
        }
        // 删除离群点
        float stdev = 3.0; // 标准差
        while (stdev > 2.5 && b) {
            int sum = 0, num = 0;
            stdev = 0.0;

            // 计算平均数
            for (int i = 0; i < pt.size(); i++) {
                if (depth[i] > 0) {
                    //std::cout << depth[it->first][i] << std::endl;
                    sum += depth[i];
                    num++;
                }
            }

            // 计算标准差
            float mean = (float) sum / num;
            for (int i = 0; i < pt.size(); i++) {
                if (depth[i] > 0) {
                    stdev += (depth[i] - mean) * (depth[i] - mean);
                }
            }
            stdev = sqrt(stdev / num);

            for (int i = 0; i < pt.size(); i++) {
                if (depth[i] > 0) {
                    if (abs(depth[i] - mean) > stdev)
                        depth[i] = -1.0;
                }
            }
            //std::cout << std::endl << "sum:" << sum << "  num:" << num << "  mean:" << mean << "  stdev:" << stdev << std::endl;
        }

        /*
        for (int i = 0; i < pt.size(); i++) {
            if (depth[i] > 0) {
                std::cout << " | " << depth[i];
            }
        }
         std::cout << std::endl << std::endl;
         */


    }

    std::vector<cv::Point3f> InstanceTracking::ComputePoses(std::map<int, std::vector<cv::Point2f>> &ptCur, std::map<int, std::vector<cv::Point2f>> &ptLast, cv::Mat Tcw,
                                        std::map<int, std::vector<int>> &OptFlowID, std::map<int, std::vector<float>> &depth,std::map<int, cv::Mat>& poses){
        //std::cout << "ptCur:" << ptCur.size() << "  ptLast:" << ptLast.size() << std::endl;

        //std::cout << " OptFlowID:" << OptFlowID.size() << " depth:" << depth.size() << std::endl;

        std::vector<cv::Point3f> vP3d;

        // 遍历当前帧的特帧点
        for(auto it=ptCur.begin(),itend=ptCur.end(); it!=itend; it++){
            int label=it->first;
            std::vector<cv::Point2f> vp=it->second;
            cv::Mat Mod = cv::Mat::eye(4,4,CV_32F);
            int N = vp.size();
            //std::cout << " N:" << N << std::endl;

            if(N<20)
                continue;

            std::vector<cv::Point2f> cur_2d;
            std::vector<cv::Point3f> pre_3d;
            // 将前一帧特征点恢复成3d点
            for(int i=0; i<N; i++){
                int id = OptFlowID[label][i];
                float z = depth[label][id];

                if(z>0)
                {
                    //std::cout << z << "  ";
                    const float u = ptLast[label][id].x;
                    const float v = ptLast[label][id].y;

                    const float x = (u-mcx)*z/mfx;
                    const float y = (v-mcy)*z/mfy;
                    cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

                    // using ground truth
                    const cv::Mat Rlw = Tcw.rowRange(0,3).colRange(0,3);
                    const cv::Mat Rwl = Rlw.t();
                    const cv::Mat tlw = Tcw.rowRange(0,3).col(3);
                    const cv::Mat twl = -Rlw.t()*tlw;

                    cv::Mat x3D_p = Rwl*x3Dc+twl;

                    cv::Point3f tmp_3d;
                    tmp_3d.x = x3D_p.at<float>(0);
                    tmp_3d.y = x3D_p.at<float>(1);
                    tmp_3d.z = x3D_p.at<float>(2);
                    //pre_3d[i] = tmp_3d;

                    cur_2d.push_back(vp[i]);
                    pre_3d.push_back(tmp_3d);

                    vP3d.push_back(tmp_3d);
                }
            }
            //std::cout << std::endl << "-------" << std::endl;

            // 开始计算
            // distortion coefficients
            cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);

            // output
            cv::Mat Rvec(3, 1, CV_64FC1);
            cv::Mat Tvec(3, 1, CV_64FC1);
            cv::Mat d(3, 3, CV_64FC1);
            cv::Mat inliers;

            //std::cout << "=================" << std::endl;
            //std::cout << "label:" << label << " matches:" << pre_3d.size() << std::endl << std::endl;
            //std::cout << "=================" << std::endl;

            if(pre_3d.size()<8)
                continue;

            // solve
            int iter_num = 500;
            double reprojectionError = 0.4, confidence = 0.98; // 0.3 0.5 1.0
            cv::solvePnPRansac(pre_3d, cur_2d, mK, distCoeffs, Rvec, Tvec, false,
                               iter_num, reprojectionError, confidence, inliers, cv::SOLVEPNP_AP3P); // AP3P EPNP P3P ITERATIVE DLS

            cv::Rodrigues(Rvec, d);

            // assign the result to current pose
            Mod.at<float>(0,0) = d.at<double>(0,0); Mod.at<float>(0,1) = d.at<double>(0,1); Mod.at<float>(0,2) = d.at<double>(0,2); Mod.at<float>(0,3) = Tvec.at<double>(0,0);
            Mod.at<float>(1,0) = d.at<double>(1,0); Mod.at<float>(1,1) = d.at<double>(1,1); Mod.at<float>(1,2) = d.at<double>(1,2); Mod.at<float>(1,3) = Tvec.at<double>(1,0);
            Mod.at<float>(2,0) = d.at<double>(2,0); Mod.at<float>(2,1) = d.at<double>(2,1); Mod.at<float>(2,2) = d.at<double>(2,2); Mod.at<float>(2,3) = Tvec.at<double>(2,0);

            std::cout << "label:" << label << " matches:" << pre_3d.size() << std::endl;
            //std::cout << "label:" << label << " matches:" << pre_3d.size() << std::endl << Mod << std::endl;
        }

        std::cout << "=======================" << std::endl;
        return vP3d;
    }

    void InstanceTracking::ComputeLabelAndBoundary(const cv::Mat &imgSem, std::set<int> &Label,
                                                   std::map<int, std::vector<cv::Point2i>> &Boundary) {
        for (int i = 2; i < imgSem.rows - 1; i++) {
            for (int j = 3; j < imgSem.cols - 2; j++) {
                // kitti360
                int label = imgSem.at<char16_t>(i, j);
                if (label > 26000 && label < 27000) {
                    //std::cout << label << std::endl;
                    if (imgSem.at<char16_t>(i - 1, j - 2) == label && imgSem.at<char16_t>(i, j - 2) == label &&
                        imgSem.at<char16_t>(i + 1, j - 2) == label &&
                        imgSem.at<char16_t>(i - 1, j - 1) == label && imgSem.at<char16_t>(i, j - 1) == label &&
                        imgSem.at<char16_t>(i + 1, j - 1) == label &&
                        imgSem.at<char16_t>(i - 1, j) == label && imgSem.at<char16_t>(i + 1, j) == label &&
                        imgSem.at<char16_t>(i - 1, j + 1) == label && imgSem.at<char16_t>(i, j + 1) == label &&
                        imgSem.at<char16_t>(i + 1, j + 1) == label &&
                        imgSem.at<char16_t>(i - 1, j + 2) == label && imgSem.at<char16_t>(i, j + 2) == label &&
                        imgSem.at<char16_t>(i + 1, j + 2) == label) {
                        Label.insert(imgSem.at<char16_t>(i, j));
                        auto it = Boundary.find(label);
                        if (it != Boundary.end()) {
                            cv::Point2i &p1 = it->second[0];
                            cv::Point2i &p2 = it->second[1];

                            if (p1.x > j)
                                p1.x = j;
                            else if (p2.x < j)
                                p2.x = j;

                            if (p1.y > i)
                                p1.y = i;
                            else if (p2.y < i)
                                p2.y = i;

                        } else {
                            cv::Point2i p1, p2;
                            p1.x = j;
                            p1.y = i;
                            p2.x = j;
                            p2.y = i;

                            Boundary.insert(std::pair<int, std::vector<cv::Point2i>>(label, {p1, p2}));
                        }
                    }
                }
            }
        }
    }

    void InstanceTracking::DrawBoundary(const cv::Mat &img, const std::map<int, std::vector<cv::Point2i>> &boundary) {
        cv::Mat im = img.clone();

        cvtColor(im, im, cv::COLOR_GRAY2BGR);
        //std::cout << im.type() << std::endl;
        cv::Point2i p1, p2;
        for (auto it = boundary.begin(), itend = boundary.end(); it != itend; it++) {
            p1 = it->second[0];
            p2 = it->second[1];
            cv::rectangle(im, p1, p2, cv::Scalar(0, 255, 0), 2);

            std::string label = std::to_string(it->first);
            cv::putText(im, label, p1, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);
        }

        cv::imshow("label", im);
        //cv::waitKey(1000);

        // save
        //cv::imwrite("/home/whd/SLAM/pic20231017/label_boundary2.png", im);

    }

}