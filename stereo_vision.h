#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QThread>
#include <QImage>
#include <QDir>

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

    bool loadRemapFile(int cam_focal_length, double base_line);

    bool rectifyImage();

    void stereoMatch();

    // capture from camera
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    // cv::Mat type capture image
    cv::Mat cap_L;
    cv::Mat cap_R;

    // RGB type for displaying
    cv::Mat img_L;
    cv::Mat img_R;

    // status
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_capture;                    // under capturing or not
    bool fg_end;                        // interrupt or not
    bool fg_calib_loaded;               // load the calibration files or not

    // stereo calibration stuffs
    QDir path_calib;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];
    cv::Mat img_r_L;
    cv::Mat img_r_R;

    // correspondence matching method
    cv::Ptr<cv::StereoSGBM> sgbm;

    // disparity image
    cv::Mat disp_raw;
    cv::Mat disp;

protected:
    void run();

signals:
    void sendImages(const cv::Mat &img_l, const cv::Mat &img_r, const cv::Mat &disp);
};

#endif // STEREO_VISION_H
