//
// Created by whd on 23-10-16.
//

#include "InstanceTracking.h"

namespace ORB_SLAM2 {
    InstanceTracking::InstanceTracking(float bf) : mbf(bf) {}

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

// new: 返回分类的fast角点，用于追踪
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

        //cv::imshow("tracked by opencv", img2_CV);
        //cv::imwrite("/home/whd/SLAM/pic20231017/optical1.png", img2_CV);
        //cv::waitKey(0);
    }

// 1017证明：每个图像分开光流与整体光流效果一样
    void InstanceTracking::OpticalFlowPyrLK(const cv::Mat &imgCur, const cv::Mat &imgLast, const cv::Mat &imgSem,
                                            std::map<int, std::vector<cv::Point2f>> &ptCur,
                                            const std::map<int, std::vector<cv::Point2f>> &ptLast) {

        std::map<int, std::vector<cv::Point2f>> ptCur_temp;
        // 可视化参数
        std::vector<cv::Point2f> pt_show_cur, pt_show_last;

        //std::cout << "========================" << std::endl;
        for (auto it = ptLast.begin(), itend = ptLast.end(); it != itend; it++) {
            ptCur_temp.insert(std::pair<int, std::vector<cv::Point2f>>(it->first, {}));

            std::vector<uchar> status;
            std::vector<float> error;
            std::vector<cv::Point2f> pt_cur;
            //std::cout << "label:" << it->first << " " << it->second.size() << std::endl;
            cv::calcOpticalFlowPyrLK(imgLast, imgCur, it->second, pt_cur, status, error);
            for (int i = 0; i < status.size(); i++) {
                if (status[i]) {
                    if (imgSem.at<char16_t>(pt_cur[i].y, pt_cur[i].x) == it->first) {
                        ptCur_temp[it->first].push_back(pt_cur[i]);
                        pt_show_cur.push_back(pt_cur[i]);
                        pt_show_last.push_back(it->second[i]);
                    }

                }
            }
        }

        /// 删除fast角点小于5个的label
        for (auto it = ptCur_temp.begin(), itend = ptCur_temp.end(); it != itend; it++) {
            if (it->second.size() > 5)
                ptCur.insert(std::pair<int, std::vector<cv::Point2f>>(it->first, it->second));
        }

        /// 可视化
        cv::Mat img2_CV;
        cv::cvtColor(imgCur, img2_CV, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < pt_show_cur.size(); i++) {
            cv::circle(img2_CV, pt_show_cur[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt_show_last[i], pt_show_cur[i], cv::Scalar(0, 250, 0));
        }

        cv::imshow("tracked by opencv", img2_CV);
        //cv::imwrite("/home/whd/SLAM/pic20231017/optical1.png", img2_CV);
        cv::waitKey(200);
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

    void InstanceTracking::ComputeDisp(const cv::Mat &imgL, const cv::Mat &imgR, cv::Mat &img_out, int SADWindowSize,
                                       int MinDisparity, int NumDisparities) {

        SGBM->setBlockSize(SADWindowSize);
        int p1 = 8 * mChannels * SADWindowSize * SADWindowSize;
        int p2 = 32 * mChannels * SADWindowSize * SADWindowSize;
        SGBM->setP1(p1);
        SGBM->setP2(p2);

        SGBM->setMinDisparity(MinDisparity);
        SGBM->setNumDisparities(NumDisparities);

        cv::Mat temp;
        SGBM->compute(imgL, imgR, temp);

        temp.convertTo(temp, CV_32F, 1.0 / 16);
        img_out = cv::Mat::zeros(temp.size(), CV_8UC1);
        cv::normalize(temp, img_out, 0, 255, cv::NORM_MINMAX);
        cv::convertScaleAbs(img_out, img_out);

        // show
        //cv::imshow("img_out", img_out);
        //cv::waitKey(0);
    }

// new
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

        //cv::Mat imgDisp = cv::Mat::zeros(temp.size(), CV_8UC1);
        //cv::normalize(temp,imgDisp, 0, 255 ,cv::NORM_MINMAX);
        //cv::convertScaleAbs(imgDisp,imgDisp);

        //std::cout << "type: " << temp.type() << std::endl;

        ComputeDepth(temp, pt, depth);

        // show
        //cv::imshow("imgDisp", imgDisp);
        //cv::waitKey(0);
    }

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
        cv::waitKey(1000);

        // save
        //cv::imwrite("/home/whd/SLAM/pic20231017/label_boundary2.png", im);

    }
}