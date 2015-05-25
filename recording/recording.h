#ifndef RECORDING_H
#define RECORDING_H

#include <QString>
#include <QDateTime>
#include <QFileDialog>
#include <QDir>
extern QDir project_path;

#include <opencv2/opencv.hpp>

#include "videorecord.h"
#include "textrecord.h"

enum RECORD_TYPE {
    VIDEO,
    TXT,
    ALL
};


class recording
{
public:
    recording(int img_h, int img_w);

    ~recording();

    void setParentFolder(QString folder);

    void setRecordType(int record_type);

    //**// It's fixed but where to use? 2015.05.25 / can't work 2015.04.06
    void setBasicInfo(cv::VideoCapture &cap);

    bool recordData(cv::Mat img);
    bool recordData(std::string data);

    QString loadData();

    void start(RECORD_TYPE type);

    void stop();

    QString save_folder;

    videoRecord *vr;

    textRecord *tr;

private:
    void createFolder();

    QDir cwd;

    QDir parent_dir;

    QString parent_folder;

    QDateTime t_now;

    QString t_save;

    int record_type;

    bool fg_parent_existed;

    bool fg_folder_existed;
};

#endif // RECORDING_H
