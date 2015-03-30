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
    ex = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
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
        if (!writer.isOpened())
            return false;
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
    cap.release();
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
            pixel->val[0] = pixel_1.val[2];
            pixel->val[1] = pixel_1.val[1];
            pixel->val[2] = pixel_1.val[0];
            pixel_2 = img_2.at<cv::Vec3b>(r, c);
            pixel = &img_merge->at<cv::Vec3b>(r, c + s.width);
            pixel->val[0] = pixel_2.val[2];
            pixel->val[1] = pixel_2.val[1];
            pixel->val[2] = pixel_2.val[0];
        }
    }
}

bool videoRecord::segmentTwoImages(cv::Mat *img_1, cv::Mat *img_2, cv::Size s)
{
    if (!cap.isOpened())
        return false;
    cv::Mat img_merge;
    cap >> img_merge;
    if (img_merge.empty())
        return false;

    for (int r = 0 ; r < s.height; r++) {
        cv::Vec3b *pixel_1, *pixel_2;
        cv::Vec3b pixel;
        for (int c = 0 ; c < s.width; c++) {
            pixel = img_merge.at<cv::Vec3b>(r, c);
            pixel_1 = &img_1->at<cv::Vec3b>(r, c);
            pixel_1->val[0] = pixel.val[2];
            pixel_1->val[1] = pixel.val[1];
            pixel_1->val[2] = pixel.val[0];
            pixel = img_merge.at<cv::Vec3b>(r, c + s.width);
            pixel_2 = &img_2->at<cv::Vec3b>(r, c);
            pixel_2->val[0] = pixel.val[2];
            pixel_2->val[1] = pixel.val[1];
            pixel_2->val[2] = pixel.val[0];
        }
    }

    return true;
}

void videoRecord::videoPath()
{
    checkFolder();
    QString file = QFileDialog::getOpenFileName(0, "Load video", save_path.path(), "Video files (*.avi)");
    cap.open(file.toStdString());    
}
