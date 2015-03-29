#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTime>
#include <QLabel>
#include <QTableView>
#include <QKeyEvent>
#include <QKeySequence>
#include <QtConcurrent/QtConcurrent>
#include <QDir>
extern QDir project_path;

#include <iostream>
#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// camera calibration
#include "calibrationform.h"

// Radar ESR controller
#include "radarcontroller.h"

static bool fg_running;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QTime t_proc_sv, t_proc_lrf, t_proc_lrf_buf, t_proc_radar;

    void reportError(QString part, QString level, QString content);

    void report(QString);

    bool cwdIsProjectFolder();               // find the project folder for loading files

    // Read params ============
    bool fg_param_loaded;               // check wether the paramerters are loaded

    // file in & out
    stereo_vision::camParam* fin_cam_param;
    stereo_vision::matchParamSGBM* fin_SGBM;
    stereo_vision::matchParamBM* fin_BM;

    void paramRead();                   // read params from basic_param.yml

    void readFromTxt(QString file_name, cv::Mat *output);   // read image data from text
    void paramWrite();
    // ======================== End

    // Radar ESR ==============
    RadarController* rc;

    bool fg_retrieving;

    // tableView used
    QStandardItemModel* model_radar;

    void radarDisplayTopViewBG();
    // ======================== End

    // Stereo vision ==========
    stereo_vision* sv;

    bool fg_capturing;

    void camOpen();

    void camStop() {}

    void svDisplayTopViewBG();
    // ======================== End

    // Stereo vision param ====
    stereoMatchParamForm *form_smp;

    bool fg_form_smp_alloc;

    void retrieveMatchParam();
    // ======================== End

    // Laser range finder =====
    lrf_controller* lrf;

    bool fg_acquiring;

    bool fg_buffering;

    bool fg_lrf_record;
    bool fg_lrf_record_quit;

    FILE* fp1;

    void lrfClearData();

    bool lrfReadData();

    cv::Mat display_lrf;

    std::vector<double> lrf_temp;
    std::vector<std::vector<double> > display_lrf_3D;
    // ======================== End

    // Fusion =================
    // topview
    int detection_range_pixel;                      // fused topview radius (pixel)

    float detection_range;                          // real detection range radius (cm)

    float ratio;                                    // scaling ratio of real to fused topview (cm -> pixel)

    float max_detection_range;                      // max. detection range in all sensors (cm)

    float min_detection_range;                      // min. detection range in all sensors (cm)

    int thickness;                                  // object params on topview

    int font;

    int font_size;

    int font_thickness;

    // information of objects detected by different sensors on fused topview
    struct sensorInformation {
        cv::Point pos;                              // sensor's position based on the vehicle view

        cv::Scalar color;                           // sensor's color on the topview

        sensorInformation() {
            pos = cv::Point(-1, -1);

            color = cv::Scalar(0, 0, 0);
        }
    };

    sensorInformation *sensors;
    // sensors:
    // [0] = stereo vision
    // [1] = radar ESR
    // [2] = laser rangefinder

    struct vehicleInfo {
        cv::Point VCP;                              // fused topview vehicle current position (pixel)

        int width;                                  // average vehicle's size (cm)

        int length;

        cv::Rect rect;

        cv::Scalar color;                           // vehicle color
    } vehicle;

    cv::Mat fused_topview_BG;

    cv::Mat fused_topview;

    void initialFusedTopView();

    void updateFusedTopView();

    void drawFusedTopView(stereo_vision::objInformation *d_sv, RadarController::ESR_track_object_info *d_radar);

    void pointTransformTopView(cv::Point sensor_pos, float range, float angle, cv::Point *output);

    int gap = 500;

    void zoomOutFusedTopView();

    void zoomInFusedTopView();
    // ======================== End

    // Thread control =========
//    bool fg_running;

    QFutureSynchronizer<void> sync;
    QFuture<bool> f_sv;
    QFuture<bool> f_lrf;
    QFuture<void> f_lrf_buf;
    QFuture<void> f_radar;
    QFuture<void> f_fused;
//    QFutureWatcher<void> fw_sv;
//    QFutureWatcher<void> fw_lrf;
//    QFutureWatcher<void> fw_lrf_buf;

    void threadCheck();

    void threadBuffering();

    void threadProcessing();
    // ======================== End

    // Camera calibration =====
    calibrationForm *form_calib;
    bool fg_form_calib_alloc;
    // ======================== End

    // Mouse control ==========
    QString mouse_info;
    // ======================== End

signals:
    // Camera calibration =====
    void sendBasicInfo(int focal_length, double base_line);
    void sendImage(cv::Mat *img);
    void sendImages(cv::Mat *img_L, cv::Mat *img_R);
    // ======================== End

    // Stereo vision param ====
    // ======================== End

private slots:

    // Laser range finder =====
    void on_pushButton_lrf_open_clicked();

    void lrfDisplay(double *lrf_data, cv::Mat *display_lrf);
    // ======================== End

    // Stereo vision ==========
    void on_pushButton_cam_open_clicked();

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();

    void on_checkBox_do_calibration_clicked(bool checked);

    void on_checkBox_do_depth_clicked(bool checked);

    void svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj);
    // ======================== End

    // Stereo vision param ====
    void closeFormSmp(void);

    void on_radioButton_BM_clicked();

    void on_radioButton_SGBM_clicked();

    void on_pushButton_stereo_match_param_clicked();

    void on_comboBox_camera_focal_length_currentIndexChanged(int index);

    void on_lineEdit_base_line_returnPressed();
    // ======================== End

    // Camera calibration =====
    void on_pushButton_camera_calibration_clicked();

    void closeEvent(QCloseEvent *);

    void requestImage(char CCD);

    void closeFormCalib(void);
    // ======================== End

    // Radar ESR ==============
    void radarDisplay(int detected_obj, cv::Mat *img, cv::Mat *topview);
    // ======================== End

    // Mouse control ==========
    void mouseXY(int x, int y);
    // ======================== End

    void on_pushButton_lrf_request_clicked();

    void on_pushButton_lrf_retrieve_clicked();

    void on_pushButton_lrf_stop_clicked();

    void on_pushButton_lrf_request_ONCE_clicked();

    void on_lineEdit_sv_focal_length_returnPressed();

    void on_pushButton_lrf_record_data_clicked();

    void on_pushButton_sv_record_data_clicked();

    void on_pushButton_lrf_record_stop_clicked();

    void on_pushButton_sv_read_images_clicked();

    void on_pushButton_sv_read_disp_clicked();

    void on_pushButton_lrf_read_range_clicked();

    void on_pushButton_lrf_read_range_2_clicked();

    void on_horizontalSlider_2_sliderReleased();

    void on_horizontalSlider_sliderReleased();

    void on_pushButton_lrf_request_2_clicked();

    void on_pushButton_lrf_retrieve_2_clicked();

    void on_pushButton_lrf_stop_2_clicked();

    void on_pushButton_radar_open_clicked();

    void on_pushButton_radar_write_clicked();

    void on_pushButton_radar_bus_on_clicked();

    void on_pushButton_radar_bus_off_clicked();

    void on_checkBox_sv_topview_clicked(bool checked);

    void on_checkBox_radar_topview_clicked(bool checked);

    void on_pushButton_stop_all_clicked();

    void on_spinBox_radar_topview_r_valueChanged(int arg1);

    void on_spinBox_radar_topview_c_valueChanged(int arg1);

    void on_spinBox_topview_r_valueChanged(int arg1);

    void on_spinBox_topview_c_valueChanged(int arg1);

    void wheelEvent(QWheelEvent *ev);

    void keyPressEvent(QKeyEvent *ev);

    void on_checkBox_pseudo_color_clicked(bool checked);

    void on_checkBox_topview_plot_points_clicked(bool checked);

    void on_checkBox_sv_reproject_clicked(bool checked);
    void on_pushButton_sv_record_clicked();
};

    // Mouse control ==========
class MouseLabel : public QLabel
{
    Q_OBJECT
public:
    MouseLabel(QWidget* parent = 0);

signals:
    void mXY(int x, int y);

protected:
    virtual void mouseMoveEvent(QMouseEvent *e);
};

#endif // MAINWINDOW_H
