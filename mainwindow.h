#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTime>
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

private slots:

    // Laser range finder =====
    void on_pushButton_lrf_open_clicked();

    void on_pushButton_lrf_display_clicked();

    void lrfDisplay();
    // ======================== End

    // Stereo vision ==========
    void on_pushButton_cam_open_clicked();

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();

    void on_checkBox_do_calibration_clicked(bool checked);

    void on_checkBox_do_depth_clicked(bool checked);

    void svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp);
    // ======================== End

    void on_pushButton_4_clicked();

    void on_pushButton_camera_calibration_clicked();

    void closeEvent(QCloseEvent *);

    // Camera calibration =====
    void requestImage(char CCD);
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

    QTime t_proc;

    void reportError(QString part, QString level, QString content);

    void report(QString);

    bool projectFolder();

    void paramRead();

    void paramWrite();

    // Stereo vision ==========
    stereo_vision* sv;

    bool fg_capturing;

    void camOpen();

    void camStop() {}

    // psuedo-color table
    QImage *color_table;

    int min_distance = 200 ;
    int max_distance = 3000 ;

    cv::Mat disp_pseudo;

    void pseudoColorTable();
    // ======================== End

    // Stereo vision param ====
    stereoMatchParamForm *form_smp;

    bool fg_form_smp_alloc;
    // ======================== End

    // Laser range finder =====
    lrf_controller* lrf;

    bool fg_acquiring;

    bool fg_buffering;

    double lrf_data[LENGTH_DATA];

    void lrfClearData();

    bool lrfReadData();

    cv::Mat display_lrf;
    // ======================== End

    // Thread control =========
//    bool fg_running;

    QFutureSynchronizer<void> sync;
    QFuture<bool> f_sv;
    QFuture<bool> f_lrf;
    QFuture<void> f_lrf_buf;
//    QFutureWatcher<void> fw_sv;
//    QFutureWatcher<void> fw_lrf;
//    QFutureWatcher<void> fw_lrf_buf;

    void threadbuffering();

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

    // Laser range finder =====
    void updateGUI();
    // ======================== End
    // Mouse control ==========
    void mouseXY(int x, int y);
    // ======================== End

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
