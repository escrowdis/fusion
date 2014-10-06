#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QThread>
#include <QImage>
#include <QDir>
#include <QFile>

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

    void close();

    bool loadRemapFile(int cam_focal_length, double base_line);

    void stereoVision();

    // RGB images for displaying
    cv::Mat img_L;
    cv::Mat img_R;

    // calibrated images
    cv::Mat img_r_L;
    cv::Mat img_r_R;

    // status
    bool fg_calib;                      // check whether the calibration button is checked

    // disparity image
    cv::Mat disp;

protected:
    void run();

signals:
    void sendImages(const cv::Mat &img_l, const cv::Mat &img_r, const cv::Mat &disp);

private:
    void paramInitialize();

    void camCapture();

    bool rectifyImage();

    void stereoMatch();

    // status
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_capture;                    // under capturing or not
    bool fg_end;                        // interrupt or not
    bool fg_calib_loaded;               // load the calibration files or not

    // capture from camera
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    // cv::Mat type capture image
    cv::Mat cap_L;
    cv::Mat cap_R;

    // stereo calibration stuffs
    QDir remap_path;
    QString remap_file;
    int cam_focal_length;
    double base_line;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];

    // correspondence matching method
    cv::Ptr<cv::StereoSGBM> sgbm;
    cv::Mat img_sgbm_L;
    cv::Mat img_sgbm_R;

    // disparity image
    cv::Mat disp_raw;
};

#endif // STEREO_VISION_H
