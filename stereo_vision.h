#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QTime>
#include <QFile>
#include <QDir>
extern QDir project_path;

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock;

#include <stack>

#include <opencv2/opencv.hpp>

// topview
#include "topview.h"

#define IMG_W 640
#define IMG_H 480
#define IMG_DIS_W 320
#define IMG_DIS_H 240
#define IMG_DIS_DISP_W 320
#define IMG_DIS_DISP_H 240

namespace SV {

enum STEREO_MATCH{
    SGBM,
    BM
};

enum INPUT_SOURCE {
    CAM,
    IMG
};

}

class stereo_vision : public QObject, public TopView
{
    Q_OBJECT

public:
    explicit stereo_vision();

    ~stereo_vision();

    int match_mode;

    bool open(int device_index_L, int device_index_R);

    bool isOpened() {return fg_cam_opened;}

    void close();

    bool loadRemapFile(int cam_focal_length, double base_line);

    bool dataExec();

    // RGB images for displaying
    cv::Mat img_L;
    cv::Mat img_R;

    // calibrated images
    cv::Mat img_r_L;
    cv::Mat img_r_R;

    // blob objects image
    cv::Mat img_detected;

    // status
    int input_mode;
    bool fg_calib;                      // check whether the calibration button is checked
    bool fg_stereoMatch;                // check whether do the correspondence matching
    bool fg_pseudo;                     // chekc wether pesudo the disparity image
    bool fg_topview;                    // check wether project to topview
    bool fg_reproject;                  // check wether re-project detected objects to image
    bool fg_topview_plot_points;        // check wethere to plot each points on the topview

    // disparity image
    cv::Mat disp_raw;
    cv::Mat disp;
    cv::Mat disp_pseudo;

    // depth estimatiom
    struct camParam
    {
        double param_r;
        double focal_length;
        int cam_focal_length;
        double base_line;
    }cam_param;

    // data - stereo vision ========
    struct StereoData
    {
        short int disp;
        // World coordinate system (WCS)
        int X;
        int Y;
        int Z;
        std::pair<int, int> grid_id;    // located cell in topview

        StereoData() {
            disp = -1;
            X = -1;
            Y = -1;
            Z = -1;
            grid_id = std::pair<int, int>(-1, -1);
        }
    };

    StereoData** data;
    // ============================= End

    // Stereo vision params ========
    void matchParamInitialize(int type);

    void updateParamsSmp();

    struct matchParamSGBM
    {
        int pre_filter_cap;
        int SAD_window_size;
        int min_disp;
        int num_of_disp;
        int uniquenese_ratio;
        int speckle_window_size;
        int speckle_range;
    }param_sgbm;

    struct matchParamBM
    {
        int pre_filter_size;
        int pre_filter_cap;
        int SAD_window_size;
        int min_disp;
        int num_of_disp;
        int texture_thresh;
        int uniquenese_ratio;
        int speckle_window_size;
        int speckle_range;
    }param_bm;

    // ============================= End

    // object information ==========
    int obj_nums;                       // maximum object detection amount

    struct objInformation
    {
        bool labeled;                   // filtered object

        std::pair<int, int> tl;         // Top left (row, col)

        std::pair<int, int> br;         // Bottom right (row, col)

        std::pair<int, int> center;     // Center point of object in image (row, col)

        float angle;                    // orientation degree. Middle is zero. (degree)

        float range;                    // (cm)

        int avg_Z;                      // average depth

        int pts_num;

        int closest_count;              // smaller number represents closer to vehicle

        objInformation() {
            labeled = false;
            tl = std::pair<int, int>(-1, -1);
            br = std::pair<int, int>(-1, -1);
            center = std::pair<int, int>(-1, -1);
            angle = 0.0;
            range = 0.0;
            avg_Z = 0;
            pts_num = 0;
            closest_count = 0;
        }
    };

    objInformation* objects;            // filtered objects
    objInformation* objects_display;

    void updateDataFroDisplay();
    // ============================= End

private:
    void resetOpen(int device_index_L, int device_index_R);

    void camCapture();

    bool rectifyImage();

    void depthCalculation();

    void stereoMatch();

    // status
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_cam_opened;
    bool fg_calib_loaded;               // load the calibration files or not
    QTime t;                            // control gui not to update too fast
    int time_gap;

    // capture from camera
    int device_index_L;
    int device_index_R;
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    // cv::Mat type capture image
    cv::Mat cap_L;
    cv::Mat cap_R;

    // stereo calibration stuffs
    QDir remap_path;
    QString remap_folder;
    QString remap_file;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];

    // correspondence matching method
    cv::Ptr<cv::StereoSGBM> sgbm;
    cv::Ptr<cv::StereoBM> bm;
    int cn;

    cv::Mat img_match_L;
    cv::Mat img_match_R;

    // object params ===============
    int thick_obj_rect, radius_obj_point;

    int detected_obj;                   // detected object number

    objInformation obj_temp;            // sorting used

    void resetBlob();

    void blob(int thresh_pts_num);
    // ============================= End

    // Topview =====================
    void pointProjectTopView();

    void pointProjectImage();
    // ============================= End

private slots:
    // BM ===========================
    void change_bm_pre_filter_size(int value);

    void change_bm_pre_filter_cap(int value);

    void change_bm_sad_window_size(int value);

    void change_bm_min_disp(int value);

    void change_bm_num_of_disp(int value);

    void change_bm_texture_thresh(int value);

    void change_bm_uniqueness_ratio(int value);

    void change_bm_speckle_window_size(int value);

    void change_bm_speckle_range(int value);
    // ============================== End

    // SGBM =========================
    void change_sgbm_pre_filter_cap(int value);

    void change_sgbm_sad_window_size(int value);

    void change_sgbm_min_disp(int value);

    void change_sgbm_num_of_disp(int value);

    void change_sgbm_uniqueness_ratio(int value);

    void change_sgbm_speckle_window_size(int value);

    void change_sgbm_speckle_range(int value);
    // ============================== End

signals:
    void sendCurrentParams(std::vector<int> param);

    void updateGUI(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj);

    void setConnect(int old_mode, int new_mode);
};

#endif // STEREO_VISION_H
