#ifndef CALIBRATIONFORM_H
#define CALIBRATIONFORM_H

#include "debug_info.h"

#include <QWidget>
#include <QDateTime>
#include <QKeyEvent>
#include <QFileDialog>
#include <QDir>
extern QDir project_path;

#include "camera_calibration.h"

namespace Ui {
class calibrationForm;
}

//!
//! \brief The calibrationForm class is a little widget to do the camera calibration
//!
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

    //!
    //! \brief saveImage is related to \link MainWindow::sendImage
    //! \param img
    //!
    void saveImage(cv::Mat *img);

    //!
    //! \brief saveImages is related to \link MainWindow::sendImages
    //! \param img_L
    //! \param img_R
    //!
    void saveImages(cv::Mat *img_L, cv::Mat *img_R);

    void on_checkBox_SaveBoth_clicked(bool checked);

    //!
    //! \brief getBasicInfo to acquire camera's basic info. (related to \link MainWindow::sendBasicInfo)
    //! \param focal_length
    //! \param base_line
    //!
    void getBasicInfo(int focal_length, double base_line);

    void on_pushButton_corner_intrinsic_clicked();

    void closeEvent(QCloseEvent *);

signals:
    void closed(void);

    void requestImage(char CCD);

private:
    Ui::calibrationForm *ui;

    char CCD;                   ///< processing CCD: R, L and B.
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
