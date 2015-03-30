#ifndef VIDEORECORD_H
#define VIDEORECORD_H

#include <QString>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
extern QDir project_path;

#include <opencv2/opencv.hpp>

class videoRecord
{
public:
    videoRecord();

    ~videoRecord();

    bool fg_record;

    cv::VideoWriter writer;

    cv::VideoCapture cap;

    void getBasicInfo(cv::VideoCapture *cap);

    bool record(cv::Mat img);

    void stop();

    void videoPath();

    virtual void loadVideo() {}

    void combineTwoImages(cv::Mat *img_merge, cv::Mat img_1, cv::Mat img_2, cv::Size s);

    bool segmentTwoImages(cv::Mat *img_1, cv::Mat *img_2, cv::Size s);

private:
    // folder named by time
    QDateTime t_now;

    QDir save_path;

    QString save_folder;

    QString file_name;

    int ex;
    double fps;
    cv::Size s;

    bool fg_file_established;

    void checkFolder();

    void createVideo();
};

#endif // VIDEORECORD_H
