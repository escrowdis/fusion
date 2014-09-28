#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QThread>
#include <QDebug>

#include <opencv2/opencv.hpp>

#define IMG_W 320
#define IMG_H 240

class stereo_vision : public QThread
{
public:
    void start();

    void stop();

    stereo_vision();

    ~stereo_vision();

    bool open(int com_L, int com_R);

    void paramInitialize();

    void close();

    void camCapture(cv::Mat &img_L, cv::Mat &img_R);

    void stereoMatch(cv::Mat &img_L, cv::Mat &img_R, cv::Mat &disp8);

    void disparityCalc(cv::Mat &img_L, cv::Mat &img_R, cv::Mat &disp);

    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    cv::Mat cap_L;
    cv::Mat cap_R;

    bool fg_cam_L, fg_cam_R;
    bool fg_capture;
    bool fg_end;

    cv::Ptr<cv::StereoSGBM> sgbm;

    cv::Mat disp;
    cv::Mat disp_raw;

public slots:
    void run(cv::Mat &img_L, cv::Mat &img_R, cv::Mat &disp);
};

#endif // STEREO_VISION_H
