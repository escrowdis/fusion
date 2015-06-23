#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "../debug_info.h"

#include "opencv2/opencv.hpp"

class KalmanFilter
{
public:
    KalmanFilter();

    ~KalmanFilter();

    struct objectTrackingKF {
        cv::KalmanFilter kf_core;

        cv::Mat state;

        cv::Mat processNoise;

        cv::Mat measurement;

        cv::Mat prediction;

        cv::Mat estimated;

        cv::Point predictPt;

        cv::Point statePt;

        objectTrackingKF() {
            kf_core = cv::KalmanFilter(4, 2, 0);
            state = cv::Mat_<float>(4, 1);  // (x, y, Vx, Vy)
            processNoise = cv::Mat(4, 1, CV_32F);
            measurement = cv::Mat_<float>::zeros(2, 1);
        }
    };
};

#endif // KALMANFILTER_H
