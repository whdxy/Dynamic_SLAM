//
// Created by whd on 23-10-16.
//

#ifndef PRA_INSTANCETRACKING_H
#define PRA_INSTANCETRACKING_H
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {


    class InstanceTracking {
    public:

        InstanceTracking();
        InstanceTracking(float bf, float fx, float fy, float cx, float cy);

        /// fast
        void FastInitialization(int threshold);

        void FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::vector<cv::Point2f> &pt);

        void FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::map<int, std::vector<cv::Point2f>> &pt);

        void FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::map<int, std::vector<cv::Point2f>> &ptCur,
                   const std::map<int, std::vector<cv::Point2f>> &ptLast);

        void FastDetectFill(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                                          std::map<int, std::vector<cv::Point2f>> &pt);

        /// OpticalFlow
        void OpticalFlowPyrLK(const cv::Mat &imgCur, const cv::Mat &imgLast, const cv::Mat &imgSem,
                              std::map<int, std::vector<cv::Point2f>> &ptCur, std::map<int, std::vector<int>> &OptFlowID,
                              const std::map<int, std::vector<cv::Point2f>> &ptLast);

        /// StereoMatch
        void StereoMatchingInitialization(int Channels, int PreFilterCap, int UniquenessRatio, int SpeckleRange);

        // old
        void
        ComputeDisp(const cv::Mat &imgL, const cv::Mat &imgR, cv::Mat &img_out, int SADWindowSize, int MinDisparity,
                    int NumDisparities);

        // new
        void ComputeDisp(const cv::Mat &imgL, const cv::Mat &imgR, const std::map<int, std::vector<cv::Point2f>> &pt,
                         std::map<int, std::vector<float>> &depth, int SADWindowSize, int MinDisparity,
                         int NumDisparities);

        void ComputeDepth(const cv::Mat &imgDisp, const std::map<int, std::vector<cv::Point2f>> &pt,
                          std::map<int, std::vector<float>> &depth);


        /// Compute Poses
        std::vector<cv::Point3f> ComputePoses(std::map<int, std::vector<cv::Point2f>> &ptCur, std::map<int, std::vector<cv::Point2f>> &ptLast, cv::Mat Tcw,
                          std::map<int, std::vector<int>> &OptFlowID, std::map<int, std::vector<float>> &depth,std::map<int, cv::Mat>& poses);

        /// other
        void ComputeLabelAndBoundary(const cv::Mat &imgSem, std::set<int> &label, std::map<int, std::vector<cv::Point2i>> &boundary);

        void DrawBoundary(const cv::Mat &img, const std::map<int, std::vector<cv::Point2i>> &boundary);

        void Setbf(float bf);

    private:

        // fast
        cv::Ptr<cv::FastFeatureDetector> FAST;

        // StereoMatch
        cv::Ptr<cv::StereoSGBM> SGBM;
        int mChannels;
        int mPreFilterCap;
        int mUniquenessRatio;
        int mSpeckleRange;

        float mbf, mfx, mfy, mcx, mcy;
        cv::Mat mK;

    };


}
#endif //PRA_INSTANCETRACKING_H
