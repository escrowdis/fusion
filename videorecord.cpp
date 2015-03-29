#include "videorecord.h"

videoRecord::videoRecord()
{
    fg_record = false;

    fg_file_established = false;
}

videoRecord::~videoRecord()
{
    if (fg_record) {
        writer.release();
        fg_record = false;
    }
}
void videoRecord::checkFolder()
{
    // set folder named by time
    save_folder = "video";
    save_path = project_path;
    if (!save_path.exists(save_folder))
        save_path.mkdir(save_folder);
    save_path.cd(save_folder);
}

void videoRecord::getBasicInfo(cv::VideoCapture *cap)
{
    checkFolder();

//    ex = static_cast<int>(cap->get(cv::CAP_PROP_FOURCC));
//    fps = cap->get(cv::CAP_PROP_FPS);
    ex = -1;
    fps = 24;
    s = cv::Size(2 * cap->get(cv::CAP_PROP_FRAME_WIDTH),
                          cap->get(cv::CAP_PROP_FRAME_HEIGHT));
}

void videoRecord::createVideo()
{
    if (!fg_file_established) {
        file_name = save_path.path() + "/" + t_now.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss") + ".avi";

        if (!writer.isOpened()) {
            writer.open(file_name.toStdString(), ex, fps, s, true);
        }

        fg_file_established = true;
    }
}

bool videoRecord::record(cv::Mat img)
{
    if (!fg_file_established)
        createVideo();

    if (fg_record) {
        writer.write(img);
        return true;
    }
    return false;
}

void videoRecord::stop()
{
    fg_file_established = false;
    fg_record = false;
    writer.release();
}

void videoRecord::combineTwoImages(cv::Mat *img_merge, cv::Mat img_1, cv::Mat img_2, cv::Size s)
{
    for (int r = 0 ; r < s.height; r++) {
        cv::Vec3b pixel_1, pixel_2;
        cv::Vec3b *pixel;
        for (int c = 0 ; c < s.width; c++) {
            pixel_1 = img_1.at<cv::Vec3b>(r, c);
            pixel = &img_merge->at<cv::Vec3b>(r, c);
            *pixel = pixel_1;
            pixel_2 = img_2.at<cv::Vec3b>(r, c);
            pixel = &img_merge->at<cv::Vec3b>(r, c + s.width);
            *pixel = pixel_2;
        }
    }
}

void videoRecord::segmentTwoImages(cv::Mat *img_1, cv::Mat *img_2, cv::Size s)
{
    cv::Mat img_merge;
    cap >> img_merge;
    for (int r = 0 ; r < s.height; r++) {
        cv::Vec3b *pixel_1, *pixel_2;
        cv::Vec3b pixel;
        for (int c = 0 ; c < s.width; c++) {
            pixel = img_merge.at<cv::Vec3b>(r, c);
            pixel_1 = &img_1->at<cv::Vec3b>(r, c);
            *pixel_1 = pixel;
            pixel = img_merge.at<cv::Vec3b>(r, c + s.width);
            pixel_2 = &img_2->at<cv::Vec3b>(r, c);
            *pixel_2 = pixel;
        }
    }
}

void videoRecord::videoPath()
{
    checkFolder();
    QString file = QFileDialog::getOpenFileName(0, "Load video", save_path.path(), "Video files (*.avi)");
    cap.open(file.toStdString());
}
