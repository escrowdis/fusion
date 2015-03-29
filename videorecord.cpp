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

void videoRecord::getBasicInfo(cv::VideoCapture *cap)
{
    // set folder named by time
    save_folder = "video";
    save_path = project_path;
    if (!save_path.exists(save_folder))
        save_path.mkdir(save_folder);
    save_path.cd(save_folder);

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
