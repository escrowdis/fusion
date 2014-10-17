#include "stereo_vision.h"

stereo_vision::stereo_vision()
{
    device_index_L = -1;
    device_index_R = -1;
    fg_cam_L = false;
    fg_cam_R = false;
    fg_cam_opened = false;
    fg_calib_loaded = false;
    fg_calib = false;
    fg_stereoMatch = false;

    // Initialization images for displaying
    img_r_L = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    img_r_R = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    disp = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);

    matchParamInitialize(STEREO_MATCH::SGBM);
}

stereo_vision::~stereo_vision()
{
    close();
    sgbm.release();
    bm.release();
}

void stereo_vision::resetOpen(int device_index_L, int device_index_R)
{

    if (this->device_index_L != device_index_L) {
        cam_L.release();
        fg_cam_L = false;
    }
    if (this->device_index_R != device_index_R) {
        cam_R.release();
        fg_cam_R = false;
    }
}

bool stereo_vision::open(int device_index_L, int device_index_R)
{
    resetOpen(device_index_L, device_index_R);
    if (fg_cam_L && fg_cam_R)
        return true;
    if (device_index_L == device_index_R || device_index_L < 0 || device_index_R < 0)
        return false;
    if (!cam_L.isOpened()) {
        if (cam_L.open(device_index_L)) {
            if (cam_L.isOpened()) {
                cam_L.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
                cam_L.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
                fg_cam_L = true;
                this->device_index_L = device_index_L;
#ifdef debug_info_sv
                qDebug()<<"open L";
#endif
            }
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail L";
#endif
        }
    }
    if (!cam_R.isOpened()) {
        if (cam_R.open(device_index_R)) {
            if (cam_R.isOpened()) {
                cam_R.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
                cam_R.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
                fg_cam_R = true;
                this->device_index_R = device_index_R;
#ifdef debug_info_sv
                qDebug()<<"open R";
#endif
            }
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail R";
#endif
        }
    }
    if (fg_cam_L && fg_cam_R)
        fg_cam_opened = true;

    return fg_cam_opened;
}

void stereo_vision::close()
{
    if (cam_L.isOpened())
        cam_L.release();
    if (cam_R.isOpened())
        cam_R.release();
}

void stereo_vision::matchParamInitialize(int type)
{
    match_type = type;
    int SAD_window_size = 0, number_disparity = 128;  // 0
    switch (type) {
    case STEREO_MATCH::BM:
        bm = cv::createStereoBM(16, 9);
//        bm->setROI1(roi1);
//        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SAD_window_size > 0 ? SAD_window_size : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(number_disparity);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);
        break;
    case STEREO_MATCH::SGBM:
        sgbm = cv::createStereoSGBM(0, 16, 3);
        SAD_window_size = 0;
        number_disparity = 0;
        number_disparity = number_disparity > 0 ? number_disparity : ((IMG_W / 8) + 15) & -16;
        sgbm->setPreFilterCap(63);
        int sgbm_win_size = SAD_window_size > 0 ? SAD_window_size : 3;
        sgbm->setBlockSize(sgbm_win_size);

        // channel
        cn = 3;

        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(number_disparity);
        sgbm->setP1(8 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setP2(32 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
        break;
    }
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
    remap_folder = "calibrationImgs";
    remap_file = QString("My_Data_" + QString::number(cam_focal_length) + "_" + QString::number(base_line) + ".yml");
    remap_path = QDir::currentPath();
    QString current_folder = remap_path.path().section("/", -1, -1);

    if  (current_folder == "release" || current_folder == "debug")
        remap_path.cdUp();
    else if (current_folder != "Fusion")
        return fg_calib_loaded;
    remap_path.cd(remap_folder);

#ifdef debug_info_sv
    qDebug()<<"path exist: "<<remap_path.exists()<<"path: "<<remap_path.path();
#endif
    if (!remap_path.exists())
        return fg_calib_loaded;

#ifdef debug_info_sv
    qDebug() << "remap folder: " << current_folder << "\tfile: " << remap_file;
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
//    img_match_L = img_r_L.clone();
//    img_match_R = img_r_R.clone();
    cv::cvtColor(img_r_L, img_match_L, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r_R, img_match_R, cv::COLOR_BGR2GRAY);

//    cv::equalizeHist(img_match_L, img_match_L);
//    cv::equalizeHist(img_match_R, img_match_R);

//    cv::GaussianBlur(img_match_L, img_match_L, cv::Size(7, 7), 0, 0);
//    cv::GaussianBlur(img_match_R, img_match_R, cv::Size(7, 7), 0, 0);

    if (match_type == STEREO_MATCH::BM)
        bm->compute(img_match_L, img_match_R, disp_raw);
    else if (match_type == STEREO_MATCH::SGBM)
        sgbm->compute(img_match_L, img_match_R, disp_raw);
    disp_raw.convertTo(disp, CV_8U);
}

void stereo_vision::stereoVision()
{
#ifdef debug_info_sv
    qDebug()<<"run";
#endif

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

#ifdef debug_info_sv
    qDebug()<<"run"<<&img_L;
#endif
}

void stereo_vision::change_pre_filter_size(const int &value)
{
//    qDebug()<<value;
    //
}

void stereo_vision::change_pre_filter_cap(const int &value)
{
//    qDebug()<<value;
    sgbm->setPreFilterCap(value);
    qDebug()<<sgbm->getPreFilterCap();
}

void stereo_vision::change_sad_window_size(const int &value)
{
//    qDebug()<<value;
    sgbm->setP1(8 * cn * value * value);
    sgbm->setP2(32 * cn * value * value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getP1()<<sgbm->getP2();
#endif
}

void stereo_vision::change_min_disp(const int &value)
{
//    qDebug()<<value;
    sgbm->setMinDisparity(value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getMinDisparity();
#endif
}

void stereo_vision::change_num_of_disp(const int &value)
{
//    qDebug()<<value;
    sgbm->setNumDisparities(value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getNumDisparities();
#endif
}

void stereo_vision::change_texture_thresh(const int &value)
{
//    qDebug()<<value;
    //
}

void stereo_vision::change_uniqueness_ratio(const int &value)
{
//    qDebug()<<value;
    sgbm->setUniquenessRatio(value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getUniquenessRatio();
#endif
}

void stereo_vision::change_speckle_window_size(const int &value)
{
//    qDebug()<<value;
    sgbm->setSpeckleWindowSize(value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getSpeckleWindowSize();
#endif
}

void stereo_vision::change_speckle_range(const int &value)
{
//    qDebug()<<value;
    sgbm->setSpeckleRange(value);
#ifdef debug_info_cc
    qDebug()<<sgbm->getSpeckleRange();
#endif
}
