#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <opencv2/opencv.hpp>

#define IMG_W 320
#define IMG_H 240

class stereo_vision
{
public:
    stereo_vision();

    bool open(int com_L, int com_R);

    void close();

    void paramInitialize();

    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    bool fg_cam_L, fg_cam_R;

    cv::Ptr<cv::StereoSGBM> sgbm;
};

#endif // STEREO_VISION_H
