#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTime>
#include <QLabel>
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

private:
    Ui::MainWindow *ui;

    QTime t_proc;

    void reportError(QString part, QString level, QString content);

    void report(QString);

    bool projectFolder();

    void paramRead();

    void paramWrite();

    void readFromTxt(QString file_name, cv::Mat *output);

    // Stereo vision ==========
    stereo_vision* sv;

    bool fg_capturing;

    void camOpen();

    void camStop() {}

    // psuedo-color table
    QImage *color_table;

    int min_distance = 200 ;    // cm
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

    bool fg_lrf_record;
    bool fg_lrf_record_quit;
    FILE* fp1;

    void lrfClearData();

    bool lrfReadData();

    cv::Mat display_lrf;

    std::vector<double> lrf_temp;
    std::vector<std::vector<double> > display_lrf_3D;
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
    void lrfUpdateGUI();
    // ======================== End

    // Stereo vision param ====
    // ======================== End

private slots:

    // Laser range finder =====
    void on_pushButton_lrf_open_clicked();

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

    // Stereo vision param ====
    void closeFormSmp(void);

    void connectSmp(int old_mode, int new_mode);

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

    // Mouse control ==========
    void mouseXY(int x, int y);
    // ======================== End

    void on_pushButton_lrf_request_clicked();

    void on_pushButton_lrf_retrieve_clicked();

    void on_pushButton_lrf_stop_clicked();

    void on_pushButton_lrf_request_ONCE_clicked();

    void on_lineEdit_returnPressed();

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
