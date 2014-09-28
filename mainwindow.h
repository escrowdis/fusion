#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTimer>
#include <QTime>

#include <iostream>
#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"

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

    void stereoVision();

    void displaying();
    // ========================

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;


    // Laser range finder =====
    lrf_controller* lrf;

    QTimer *lrf_timer;

    bool lrf_status;

    double lrf_data[LENGTH_DATA];

    void lrfResetData();
    // ========================

    // Stereo vision ==========
    stereo_vision* sv;

    QTime* fps_time;
    int fps;

    cv::Mat cap_L;
    cv::Mat cap_R;

    cv::Mat img_L;
    cv::Mat img_R;

    cv::Mat disp;

    void camOpen();

    void camStop() {sv->stop();}
    // ========================
};

#endif // MAINWINDOW_H
