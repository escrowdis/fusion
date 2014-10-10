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
    void on_pushButton_lrf_open_clicked();

    void on_pushButton_lrf_display_clicked();

    // Laser range finder =====
    void lrfReadData();
    // ========================

    // Stereo vision ==========
    void on_pushButton_cam_open_clicked();

    void displaying(const cv::Mat &img_L, const cv::Mat &img_R, const cv::Mat &disp);
    // ========================

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();
    void on_pushButton_4_clicked();

    void on_checkBox_do_calibration_clicked(bool checked);

    void on_pushButton_camera_calibration_clicked();

    void closeEvent(QCloseEvent *);

    void on_checkBox_do_depth_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    void threadProcessing();

    // Laser range finder =====
    lrf_controller* lrf;

    bool fg_acquiring;

    QTimer *lrf_timer;

    bool lrf_status;

    double lrf_data[LENGTH_DATA];

    void lrfResetData();
    // ========================

    // Stereo vision ==========
    stereo_vision* sv;

    bool fg_capturing;

    void camOpen();

    void camCapture() {
        sv->stereoVision();
        displaying(sv->img_r_L, sv->img_r_R, sv->disp);
    }

    void camStop() {}
    // ========================

    // Thread control =========
    bool fg_running;

    QFutureSynchronizer<void> sync;
    QFuture<void> f_sv;
    QFuture<void> f_lrf;
    // ========================
};

#endif // MAINWINDOW_H
