#ifndef CALIBRATIONFORM_H
#define CALIBRATIONFORM_H

#include "debug_info.h"

#include <QWidget>
#include <QDir>
#include <QDateTime>
#include <QKeyEvent>
#include <QFileDialog>

#include "camera_calibration.h"

namespace Ui {
class calibrationForm;
}

class calibrationForm : public QWidget
{
    Q_OBJECT

public:
    explicit calibrationForm(QWidget *parent = 0);
    ~calibrationForm();

    camera_calibration *cc;

    void reset();

private slots:
    void on_pushButton_3_clicked();

    void on_pushButton_calibration_clicked();

    void mouseReleaseEvent(QMouseEvent *);

    void keyReleaseEvent(QKeyEvent *event);

    void saveImage(cv::Mat *img);

    void saveImages(cv::Mat *img_L, cv::Mat *img_R);

    void on_checkBox_SaveBoth_clicked(bool checked);

    // folder setting
    void getBasicInfo(int focal_length, double base_line);

    void on_pushButton_corner_intrinsic_clicked();

signals:
    void requestImage(char CCD);

private:
    Ui::calibrationForm *ui;

    char CCD;                   // Which CCD is on processing -> R, L
    char CCD_temp;
    int focal_length;
    double base_line;

    // folder setting
    QDir image_save_path;
    QString image_save_folder;
    // folder named by time
    QDateTime t_now;
    QString t_now_string;

    // file name setting
    QString file_save_L;
    QString file_save_R;

    // type for image saving & displaying
    const std::string image_name_R = "Right image"; // window name for displaying
    const std::string image_name_L = "Left image";
    cv::Mat img_s_L;
    cv::Mat img_s_R;

    // camera calibration
    QStringList images_L;
    QStringList images_R;

    void loadFiles(QString folder, std::vector <std::string> files[]);

    void nextCam();

    void prevCam();

};

#endif // CALIBRATIONFORM_H
