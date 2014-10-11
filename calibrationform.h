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

private slots:
    void on_pushButton_3_clicked();

    void on_pushButton_calibration_clicked();

    void keyReleaseEvent(QKeyEvent *event);

    void saveImage(const cv::Mat &img);

    void saveImages(const cv::Mat &img_L, const cv::Mat &img_R);

    void on_checkBox_SaveBoth_clicked(bool checked);

signals:
    void requestImage(const char &CCD);

private:
    Ui::calibrationForm *ui;

    char CCD;                   // Which CCD is on processing -> R, L
    char CCD_temp;

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


};

#endif // CALIBRATIONFORM_H
