#ifndef VIDEORECORD_H
#define VIDEORECORD_H

#include <QString>
#include <QDateTime>
#include <QFileDialog>

#include <opencv2/opencv.hpp>

class videoRecord
{
public:
    videoRecord();

    ~videoRecord();

    bool fileExist() {return !file_path.isEmpty();}

    //**// can't work 2015.04.06
//    void getBasicInfo(cv::VideoCapture *cap);

    void setPath(QString str, bool fg_str_is_file);

    virtual bool loadVideo(QString str, bool fg_str_is_file);

    bool record(cv::Mat img);

    void stop();

    void combineTwoImages(cv::Mat *img_merge, cv::Mat img_1, cv::Mat img_2, cv::Size s);

    bool segmentTwoImages(cv::Mat *img_1, cv::Mat *img_2, cv::Size s);

    bool fg_record;

    bool fg_data_end;

    cv::VideoWriter writer;

    cv::VideoCapture cap;

private:
    void defaultBasicInfo();

    QString file_path;

    QString file_name;

    int ex;
    double fps;
    cv::Size s;

    bool fg_file_established;

    void createVideo();
};

#endif // VIDEORECORD_H
