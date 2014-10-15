#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTimer>
#include <QTime>
#include <QtConcurrent/QtConcurrent>

#include <iostream>
#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// camera calibration
#include "calibrationform.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    // Laser range finder =====
    void on_pushButton_lrf_open_clicked();

    void on_pushButton_lrf_display_clicked();

    // ======================== End

    // Stereo vision ==========
    void on_pushButton_cam_open_clicked();

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();

    void on_checkBox_do_calibration_clicked(bool checked);

    void on_checkBox_do_depth_clicked(bool checked);

    void displaying(const cv::Mat &img_L, const cv::Mat &img_R, const cv::Mat &disp);
    // ======================== End

    void on_pushButton_4_clicked();

    void on_pushButton_camera_calibration_clicked();

    void closeEvent(QCloseEvent *);

    // Camera calibration =====
    void requestImage(const char &CCD);
    // ======================== End

    void on_radioButton_BM_clicked();

    void on_radioButton_SGBM_clicked();

    void on_pushButton_stereo_match_param_clicked();

    void on_comboBox_camera_focal_length_currentIndexChanged(int index);

    void on_lineEdit_base_line_returnPressed();

    void on_pushButton_lrf_stop_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

private:
    Ui::MainWindow *ui;

    void reportError(QString part, QString level, QString content);

    void report(QString);

    // Laser range finder =====
    lrf_controller* lrf;

    bool fg_acquiring;

    bool fg_buffering;

    double lrf_data[LENGTH_DATA];

    void lrfClearData();

    void lrfReadData(int mode);

    void lrfDisplay();
    // ======================== End

    // Stereo vision ==========
    stereo_vision* sv;

    // stereo match
    stereoMatchParamForm *form_smp;

    bool fg_capturing;

    void camOpen();

    void camCapture() {
        sv->stereoVision();
        displaying(sv->img_r_L, sv->img_r_R, sv->disp);
    }

    void camStop() {}
    // ======================== End

    // Thread control =========
    bool fg_running;

    QFutureSynchronizer<void> sync;
    QFuture<void> f_sv;
    QFuture<void> f_lrf;
    QFuture<void> f_lrf_buf;
    QFutureWatcher<void> fw_lrf_buf;

    void threadProcessing();
    // ======================== End

    // Camera calibration =====
    calibrationForm *form_calib;
    // ======================== End

signals:
    // Camera calibration =====
    void sendBasicInfo(const int &focal_length, const double &base_line);
    void sendImage(const cv::Mat &img);
    void sendImages(const cv::Mat &img_L, const cv::Mat &img_R);
    // ======================== End

};

#endif // MAINWINDOW_H
