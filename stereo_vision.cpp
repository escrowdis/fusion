#include "stereo_vision.h"

stereo_vision::stereo_vision(QObject *parent) : QThread(parent)
{
    fg_cam_L = false;
    fg_cam_R = false;
    fg_capture = false;
    fg_end = true;
    fg_calib_loaded = false;
    fg_calib = false;
    fg_stereoMatch = false;

    // Initialization images for displaying
    img_r_L = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    img_r_R = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    disp = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);

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

bool stereo_vision::loadRemapFile(int cam_focal_length, double base_line)
{
    // The folder of calibration files should be placed under project's folder
    // check whether the file has been loaded
    if (fg_calib_loaded && this->cam_focal_length == cam_focal_length && this->base_line == base_line)
        return fg_calib_loaded;

    // find files under which folder and find the folder with calibration files
    remap_file = QString("My_Data_" + QString::number(cam_focal_length) + "_" + QString::number(base_line) + ".yml");
    remap_path = QDir::currentPath();
    QString remap_last_folder = remap_path.path().section("/", -1, -1);

    if  (remap_last_folder == "release" || remap_last_folder == "debug")
        remap_path.cdUp();
    else if (remap_last_folder != "Fusion")
        return fg_calib_loaded;
    remap_path.cd("./calibrationImgs");

#ifdef debug_info_sv
    qDebug()<<"path exist: "<<remap_path.exists()<<"path: "<<remap_path.path();
#endif
    if (!remap_path.exists())
        return fg_calib_loaded;

#ifdef debug_info_sv
    qDebug() << "remap folder: " << remap_last_folder << "\tfile: " << remap_file;
#endif

    cv::FileStorage fs(QString(remap_path.path() + "/" + remap_file).toStdString().c_str(), cv::FileStorage::READ);

    if (!fs.isOpened())
        return fg_calib_loaded;

    // rewrite params
    fs["reMapLx"] >> rmapLx;
    fs["reMapLy"] >> rmapLy;
    fs["reMapRx"] >> rmapRx;
    fs["reMapRy"] >> rmapRy;
    fs["ROI0x"] >> calibROI[0].x;
    fs["ROI0y"] >> calibROI[0].y;
    fs["ROI0w"] >> calibROI[0].width;
    fs["ROI0h"] >> calibROI[0].height;
    fs["ROI1x"] >> calibROI[1].x;
    fs["ROI1y"] >> calibROI[1].y;
    fs["ROI1w"] >> calibROI[1].width;
    fs["ROI1h"] >> calibROI[1].height;
    fs.release();
    this->cam_focal_length = cam_focal_length;
    this->base_line = base_line;
    fg_calib_loaded = true;

    return fg_calib_loaded;
}

bool stereo_vision::rectifyImage()
{
    if (fg_calib_loaded) {
        cv::remap(img_L, img_r_L, rmapLx, rmapLy, cv::INTER_LINEAR);
        cv::remap(img_R, img_r_R, rmapRx, rmapRy, cv::INTER_LINEAR);
        return true;
    }
    img_r_L = img_L.clone();
    img_r_R = img_R.clone();

    return false;
}

void stereo_vision::stereoMatch()
{
    // pre-processing
    cv::cvtColor(img_r_L, img_sgbm_L, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r_R, img_sgbm_R, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(img_sgbm_L, img_sgbm_L);
    cv::equalizeHist(img_sgbm_R, img_sgbm_R);

    cv::GaussianBlur(img_sgbm_L, img_sgbm_L, cv::Size(7, 7), 0, 0);
    cv::GaussianBlur(img_sgbm_R, img_sgbm_R, cv::Size(7, 7), 0, 0);

    sgbm->compute(img_sgbm_L, img_sgbm_R, disp_raw);
    disp_raw.convertTo(disp, CV_8U);
}

void stereo_vision::stereoVision()
{
    // camera capturing
    camCapture();

    // camera calibration
    if (fg_calib)
        rectifyImage();
    else {
        img_r_L = img_L.clone();
        img_r_R = img_R.clone();
    }

    // stereo matching
    if (fg_stereoMatch)
        stereoMatch();
    else {
        disp.setTo(0);
    }
}

void stereo_vision::run()
{
    while (fg_capture) {
#ifdef debug_info_sv
        qDebug()<<"run";
#endif

        stereoVision();

#ifdef debug_info_sv
        qDebug()<<"run"<<&img_L;
#endif

        emit this->sendImages(img_r_L, img_r_R, disp);
    }
    fg_end = true;
}

