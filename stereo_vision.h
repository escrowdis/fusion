#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

//#include <QThread>
#include <QImage>
#include <QDir>
#include <QFile>

#include <opencv2/opencv.hpp>

#define IMG_W 640
#define IMG_H 480
#define IMG_DIS_W 320
#define IMG_DIS_H 240

class stereo_vision : public QObject
{
    Q_OBJECT

public:
    explicit stereo_vision();

    ~stereo_vision();

    bool open(int com_L, int com_R);

    bool isOpened() {return fg_cam_opened;}

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
    bool fg_stereoMatch;                // check whether do the correspondence matching

    // disparity image
    cv::Mat disp;

    void matchParamInitialize(int type);

    enum STEREO_MATCH{
        SGBM,
        BM
    };

private:
    void resetOpen(int com_L, int com_R);

    void camCapture();

    bool rectifyImage();

    void stereoMatch();

    // status
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_cam_opened;
    bool fg_calib_loaded;               // load the calibration files or not

    // capture from camera
    int com_L;
    int com_R;
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    // cv::Mat type capture image
    cv::Mat cap_L;
    cv::Mat cap_R;

    // stereo calibration stuffs
    QDir remap_path;
    QString remap_folder;
    QString remap_file;
    int cam_focal_length;
    double base_line;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];

    // correspondence matching method
    int match_type;
    cv::Ptr<cv::StereoSGBM> sgbm;
    cv::Ptr<cv::StereoBM> bm;
    int cn;

    cv::Mat img_match_L;
    cv::Mat img_match_R;

    // disparity image
    cv::Mat disp_raw;

private slots:
    // stereo match
    void change_pre_filter_size(const int &value);

    void change_pre_filter_cap(const int &value);

    void change_sad_window_size(const int &value);

    void change_min_disp(const int &value);

    void change_num_of_disp(const int &value);

    void change_texture_thresh(const int &value);

    void change_uniqueness_ratio(const int &value);

    void change_speckle_window_size(const int &value);

    void change_speckle_range(const int &value);
};

#endif // STEREO_VISION_H
