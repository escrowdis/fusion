#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

#include <QTime>
#include <QFile>
#include <QDir>
extern QDir project_path;

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock_sv;
extern QReadWriteLock lock_sv_data;
extern QReadWriteLock lock_sv_object;
extern QReadWriteLock lock_sv_mouse;
extern QReadWriteLock lock_f_sv;

// recording
#include "recording/recording.h"
extern recording re;

#include <stack>

#include <opencv2/opencv.hpp>
#ifdef opencv_cuda
#include "opencv2/cudastereo.hpp"
#endif

// topview
#include "topview.h"

// Sensor base
#include "sensorbase.h"

#define IMG_W 640
#define IMG_H 480
#define IMG_W_HALF 320
#define IMG_H_HALF 240
#define IMG_DIS_W 320
#define IMG_DIS_H 240
#define IMG_DIS_DISP_W 320
#define IMG_DIS_DISP_H 240

#define GROUND_RANGE 20

namespace SV {
enum STATUS {
    OK = 0,
    NO_INPUT = -1,
    NO_UPDATE = -2,
    NO_RECTIFYIMAGE = -3
};

enum STEREO_MATCH {
    SGBM,
    BM
};

enum INPUT_SOURCE {
    CAM,
    VIDEO,
    IMG
};
}

class stereo_vision : public QObject, public TopView, public SensorBase
{
    Q_OBJECT

public:
    explicit stereo_vision();

    ~stereo_vision();

    int match_mode;

    std::vector <int> match_param;

    bool open(int device_index_L, int device_index_R);

    // check if cam is opened or not
    bool isOpened() {return fg_cam_opened;}

    // close cam
    void close();

    bool loadRemapFile(int cam_focal_length, double base_line);

    int dataExec();

    int objSize() {return obj_nums;}

    int guiUpdate();

    bool fusedTopview() {return fg_topview && fg_stereoMatch;}

    void HoughLine();

    int time_proc;

    // RGB images for displaying
    cv::Mat img_L;
    cv::Mat img_R;

    // calibrated images
    cv::Mat img_r_L;
    cv::Mat img_r_R;

    // blob objects image
    cv::Mat img_detected;
    cv::Mat img_detected_display;

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
        int port_L;
        int port_R;
        double param_r;
        double focal_length;
        int cam_focal_length;
        double base_line;
        double rig_height;
    };
    camParam* cam_param;

    // data - stereo vision ========
    struct StereoData
    {
        // IMAGE -------------
        float disp;

        // World coordinate system (WCS) (cm)
        int X;
        int Y;
        int Z;

        bool ground_cand;               // recognized as ground by v-disparity algorithm

        // GRIDMAP -----------
        std::pair<int, int> grid_id;    // located cell in topview

        StereoData() {
            disp = -1;
            X = -1;
            Y = -1;
            Z = -1;
            ground_cand = false;
            grid_id = std::pair<int, int>(-1, -1);
        }
    };

    StereoData** data;

    void getMouseCursorInfo(int y, int x, float &disp, cv::Point3i &pos3D);

    void setGroundFilter(bool fg_enable) {fg_ground_filter = fg_enable;}
    // data - stereo vision ======== End

    bool modeChanged(int cur_mode) {
        return match_mode != cur_mode;
    }

    void modeChange(int cur_mode, bool fg_form_smp_update);

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
    };
    matchParamSGBM* param_sgbm;

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
    };
    matchParamBM* param_bm;
    // Stereo match params (SMP) === End

    // object params ===============
    int thick_obj_rect, radius_obj_point;
    // object params =============== End

private:
    int* LUT_grid_row;                  // depth -> grid map row
    int* LUT_grid_col;                  // image col -> grid map col
    int* LUT_depth;                     // grid map row -> depth
    int* LUT_img_col;                   // grid map col -> image col

    void createLUT();
    int corrGridRow(int k);
    int corrGridCol(int k);

    void resetOpen(int device_index_L, int device_index_R);

    bool camCapture();

    bool dataIn();

    bool rectifyImage();

    void depthCalculation();

    bool vDispCalculation();

    float point2Line(cv::Point pt, cv::Point line_1, cv::Point line_2);

    void stereoMatch();

    QTime t_p;                          // process time of all exec.

    // status ======================
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_cam_opened;                 // check if cams are opened
    bool fg_calib_loaded;               // load the calibration files or not
    QTime t;                            // control gui not to update too fast
    int time_gap;
    // status ====================== End

    // capture from camera =========
    int device_index_L;
    int device_index_R;
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;
    // capture from camera ========= End

    // cv::Mat type capture image ==
    cv::Mat cap_L;
    cv::Mat cap_R;

    // stereo calibration stuffs ===
    QDir remap_path;
    QString remap_folder;
    QString remap_file;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];

    // correspondence matching method
#ifdef opencv_cuda
    cv::Ptr<cv::cuda::StereoBM> bm;

    cv::cuda::GpuMat d_L;
    cv::cuda::GpuMat d_R;
    cv::cuda::GpuMat d_disp;
#else
    cv::Ptr<cv::StereoSGBM> sgbm;
    cv::Ptr<cv::StereoBM> bm;
#endif
    int cn;

    void updateFormParams();

    cv::Mat img_match_L;
    cv::Mat img_match_R;

    // object matching =============
    double thresh_Bha = 0.3; //**// tuned params 20150528
    double thresh_err_x = 150, thresh_err_z = 150; //**// tuned params 20150528

    // histogram calculation used
    int hist_size;
    const float *hist_ranges;

    // check if there's any object detected
    bool fg_om_existed;
    bool fg_om_prev_existed;

    // amount of objects. Default: obj_nums
    int om_size;
    int om_prev_size;

    // amount of detected object
    int om_obj_num;
    int om_prev_obj_num;

    // object's id <-> map_Bha id
    std::vector<int> map_Bha_corr_id_r;
    std::vector<int> map_Bha_corr_id_c;

    struct objMatchingInfo
    {
        SensorBase::PC pc;

        std::pair<double, double> err_pos;

        cv::Mat img;                    // object's image (cropped)

        cv::Mat H_img;

        cv::Mat H_hist;

        bool fg_Bha_check;

        // IMAGE -------------
        std::pair<int, int> center;     // Center point of object in image (row, col)
    };

    objMatchingInfo *om;
    objMatchingInfo *om_prev;

#ifdef debug_info_sv_object_matching_img
    cv::Mat comp, comp_prev;
#endif

    void resetObjMatching();

    void objectMatching();
    // object matching ============= End

    // object information ==========
    int obj_nums;                       // maximum object detection amount

public:
    struct objInformation
    {
        // SENSOR ============
        // GRIDMAP -----------
        int pts_num;

        bool labeled;                   // filtered object

        int avg_Z;                      // average depth

        int avg_X;

        int avg_Y;

        int closest_count;              // smaller number represents closer to vehicle

        cv::Point rect_tl, rect_br;

        // IMAGE -------------
        std::pair<int, int> tl;         // Top left (row, col)
        std::pair<int, int> br;         // Bottom right (row, col)
        std::pair<int, int> center;     // Center point of object in image (row, col)
        cv::Scalar color;               // object's color
        cv::Mat img;                    // object's image

        // TOPVIEW -----------
        cv::Rect rect;                  // (pixel)

        PC pc;

        // FUSED TOPVIEW -----
        cv::Rect rect_f;                // (pixel)

        cv::Point plot_pt_f;            // (pixel)

        // WCS ===============
        cv::Rect rect_world;            // (cm)

        PC pc_world;                    // (cm)

        int prev_id;

        objInformation() {
            labeled = false;
            tl = std::pair<int, int>(-1, -1);
            br = std::pair<int, int>(-1, -1);
            center = std::pair<int, int>(-1, -1);
            avg_Z = 0;
            avg_X = 0;
            avg_Y = 0;
            pts_num = 0;
            closest_count = 0;
        }
    };

    objInformation* objects_display;

private:
    objInformation* objects;            // filtered objects

    void updateDataForDisplay();
    // object information ========== End

    // object params ===============
    int detected_obj;                   // Amount of detected object

    objInformation obj_temp;            // sorting used

    void resetBlob();

    void blob(int thresh_pts_num);
    // object params =============== End

    // data - stereo vision ========
    // ground filtering
    float ground_mean_guess;

    int* ground_filter;                 // the amount of pixel within pixels' range

    float ground_mean;                  // mean of ground_filter_id

    int thresh_ground_filter;           // SD of ground_filter[]

    bool fg_ground_filter;              // do the ground filtering algorithm or not

    bool fg_vDisp;                      // if there's no frame detected
    // Stereo match params (SMP) ===
    void matchParamInitialize(int cur_mode);
    // data - stereo vision ======== End

    // Topview =====================
    void pointProjectTopView();

    void pointProjectImage();
    // Topview ===================== End

    // Object Matching =============
    void splitOneOut(int channel, cv::Mat src, cv::Mat *dst);
    // Object Matching ============= End

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
    // BM =========================== End

    // SGBM =========================
    void change_sgbm_pre_filter_cap(int value);

    void change_sgbm_sad_window_size(int value);

    void change_sgbm_min_disp(int value);

    void change_sgbm_num_of_disp(int value);

    void change_sgbm_uniqueness_ratio(int value);

    void change_sgbm_speckle_window_size(int value);

    void change_sgbm_speckle_range(int value);
    // SGBM ========================= End

signals:
    void updateGUI(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj, int current_frame_count);

    void updateForm(int mode, std::vector<int> params);

    void videoEnd();
};

#endif // STEREO_VISION_H
