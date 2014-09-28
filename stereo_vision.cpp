#include "stereo_vision.h"

stereo_vision::stereo_vision(QObject *parent) : QThread(parent)
{
    fg_cam_L = false;
    fg_cam_R = false;

    fg_capture = false;
    fg_end = true;

    paramInitialize();
}

stereo_vision::~stereo_vision()
{
    while (!fg_end) { }
    close();
    sgbm.release();
}

bool stereo_vision::open(int com_L, int com_R)
{
    if (fg_cam_L && fg_cam_R)
        return true;
    if (com_L == com_R || com_L < 0 || com_R < 0)
        return false;
    if (!cam_L.isOpened()) {
        if (cam_L.open(com_L)) {
            cam_L.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
            cam_L.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
            fg_cam_L = true;
#ifdef debug_info_sv
            qDebug()<<"open L";
#endif
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail L";
#endif
        }
    }
    if (!cam_R.isOpened()) {
        if (cam_R.open(com_R)) {
            cam_R.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
            cam_R.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
            fg_cam_R = true;
#ifdef debug_info_sv
            qDebug()<<"open R";
#endif
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail R";
#endif
        }
    }

    return fg_cam_L && fg_cam_R;
}

void stereo_vision::close()
{
    if (cam_L.isOpened())
        cam_L.release();
    if (cam_R.isOpened())
        cam_R.release();
}

void stereo_vision::paramInitialize()
{
    sgbm = cv::createStereoSGBM(0, 16, 3);

    int SAD_window_size = 0, number_disparity = 0;
    number_disparity = number_disparity > 0 ? number_disparity : ((IMG_W / 8) + 15) & -16;
    sgbm->setPreFilterCap(63);
    int sgbm_win_size = SAD_window_size > 0 ? SAD_window_size : 3;
    sgbm->setBlockSize(sgbm_win_size);

    // channel
    int cn = 3;

    sgbm->setP1(8 * cn * sgbm_win_size * sgbm_win_size);
    sgbm->setP2(32 * cn * sgbm_win_size * sgbm_win_size);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(number_disparity);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
}

void stereo_vision::start()
{
#ifdef debug_info_sv
    qDebug()<<"start";
#endif
    fg_end = false;
    fg_capture = true;

    run();
}

void stereo_vision::stop()
{
    fg_capture = false;
}

void stereo_vision::camCapture()
{
    if (cam_L.isOpened()) {
        cam_L >> cap_L;
        cv::cvtColor(cap_L, img_L, cv::COLOR_BGR2RGB);
    }
    if (cam_R.isOpened()) {
        cam_R >> cap_R;
        cv::cvtColor(cap_R, img_R, cv::COLOR_BGR2RGB);
    }
}

void stereo_vision::stereoMatch()
{
    sgbm->compute(img_L, img_R, disp_raw);
    disp_raw.convertTo(disp, CV_8U);
}

void stereo_vision::run()
{
    while (fg_capture) {
#ifdef debug_info_sv
        qDebug()<<"run";
#endif
        // camera capturing
        if (cam_L.isOpened()) {
            cam_L >> cap_L;
            cv::cvtColor(cap_L, img_L, cv::COLOR_BGR2RGB);
        }
        if (cam_R.isOpened()) {
            cam_R >> cap_R;
            cv::cvtColor(cap_R, img_R, cv::COLOR_BGR2RGB);
        }

        cv::waitKey(1);

        // stereo matching
        sgbm->compute(img_L, img_R, disp_raw);
        disp_raw.convertTo(disp, CV_8UC1);
#ifdef debug_info_sv
        qDebug()<<"run"<<&img_L;
#endif

        emit this->sendImages(img_L, img_R, disp);
//        cv::imshow("disp", disp);
    }
    fg_end = true;
}

