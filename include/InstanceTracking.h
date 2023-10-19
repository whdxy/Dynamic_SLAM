//
// Created by whd on 23-10-16.
//

#ifndef PRA_INSTANCETRACKING_H
#define PRA_INSTANCETRACKING_H
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2 {


    class InstanceTracking {
    public:
        /// fast
        InstanceTracking(float bf);

        void FastInitialization(int threshold);

        void
        FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::vector<cv::Point2f> &pt);

        void
        FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::map<int, std::vector<cv::Point2f>> &pt);

        void
        FastDetect(const cv::Mat &img, const cv::Mat &imgSem, const std::map<int, std::vector<cv::Point2i>> &boundary,
                   std::map<int, std::vector<cv::Point2f>> &ptCur,
                   const std::map<int, std::vector<cv::Point2f>> &ptLast);

        /// OpticalFlow
        void OpticalFlowPyrLK(const cv::Mat &imgCur, const cv::Mat &imgLast, const cv::Mat &imgSem,
                              std::map<int, std::vector<cv::Point2f>> &ptCur,
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

        void RemoveOutliers(std::map<int, std::vector<float>> &depth);

        /// other
        void ComputeLabelAndBoundary(const cv::Mat &imgSem, std::set<int> &label,
                                     std::map<int, std::vector<cv::Point2i>> &boundary);

        void DrawBoundary(const cv::Mat &img, const std::map<int, std::vector<cv::Point2i>> &boundary);

    private:

        // fast
        cv::Ptr<cv::FastFeatureDetector> FAST;

        // StereoMatch
        cv::Ptr<cv::StereoSGBM> SGBM;
        int mChannels;
        int mPreFilterCap;
        int mUniquenessRatio;
        int mSpeckleRange;
        float mbf;

    };


}
#endif //PRA_INSTANCETRACKING_H
