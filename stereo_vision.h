#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QThread>
#include <QImage>

#include <opencv2/opencv.hpp>

#define IMG_W 320
#define IMG_H 240

class stereo_vision : public QThread
{
    Q_OBJECT
public:
    explicit stereo_vision(QObject *parent = 0);

    ~stereo_vision();

    void start();

    void stop();

    bool open(int com_L, int com_R);

    void paramInitialize();

    void close();

    void camCapture();

    void stereoMatch();

    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    cv::Mat cap_L;
    cv::Mat cap_R;

    cv::Mat img_L;
    cv::Mat img_R;

    bool fg_cam_L, fg_cam_R;
    bool fg_capture;
    bool fg_end;

    cv::Ptr<cv::StereoSGBM> sgbm;

    cv::Mat disp;
    cv::Mat disp_raw;

protected:
    void run();

signals:
    void sendImages(const cv::Mat &img_l, const cv::Mat &img_r, const cv::Mat &disp);
};

#endif // STEREO_VISION_H
