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

    bm = cv::createStereoBM(16, 9);
    sgbm = cv::createStereoSGBM(0, 16, 3);

    match_mode = -1;
    matchParamInitialize(SV::STEREO_MATCH::SGBM);
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
    int SAD_window_size = 0, number_disparity = 128;  // 0
    switch (type) {
    case SV::STEREO_MATCH::BM:
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
    case SV::STEREO_MATCH::SGBM:
        SAD_window_size = 0; // odd number, usually from 3 to 11
        number_disparity = number_disparity > 0 ? number_disparity : ((IMG_W / 8) + 15) & -16;

        sgbm->setPreFilterCap(63);
        int sgbm_win_size = SAD_window_size > 0 ? SAD_window_size : 5;
        sgbm->setBlockSize(sgbm_win_size);

        // channel
        cn = img_r_L.channels();

        sgbm->setP1(8 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setP2(32 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(number_disparity);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
        break;
    }

    emit setConnect(match_mode, type);
    match_mode = type;
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
    remap_path = project_path;
    if (!remap_path.exists(remap_folder))
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
//    img_r_L = img_L.clone();
//    img_r_R = img_R.clone();

    return false;
}

void stereo_vision::stereoMatch()
{
    // pre-processing
    cv::cvtColor(img_r_L, img_match_L, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r_R, img_match_R, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(img_match_L, img_match_L);
    cv::equalizeHist(img_match_R, img_match_R);

    cv::GaussianBlur(img_match_L, img_match_L, cv::Size(7, 7), 0, 0);
    cv::GaussianBlur(img_match_R, img_match_R, cv::Size(7, 7), 0, 0);

    if (match_mode == SV::STEREO_MATCH::BM)
        bm->compute(img_match_L, img_match_R, disp_raw);
    else if (match_mode == SV::STEREO_MATCH::SGBM)
        sgbm->compute(img_match_L, img_match_R, disp_raw);

    disp_raw.convertTo(disp, CV_8U);

}

bool stereo_vision::stereoVision()
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
    if (fg_stereoMatch) {
        stereoMatch();
    }
    else {
        disp.setTo(0);
    }

#ifdef debug_info_sv
    qDebug()<<"run"<<&img_L<<"emit"<<&img_r_L;
#endif

    emit svUpdateGUI(&img_r_L, &img_r_R, &disp);
    return true;
}

void stereo_vision::updateParamsSmp()
{
    emit setConnect(match_mode, match_mode);

    std::vector <int> match_param;
    switch(match_mode) {
    case SV::STEREO_MATCH::BM:
        match_param.push_back(bm->getPreFilterSize());
        match_param.push_back(bm->getPreFilterCap());
        match_param.push_back(bm->getBlockSize());
        match_param.push_back(bm->getMinDisparity());
        match_param.push_back(bm->getNumDisparities());
        match_param.push_back(bm->getTextureThreshold());
        match_param.push_back(bm->getUniquenessRatio());
        match_param.push_back(bm->getSpeckleWindowSize());
        match_param.push_back(bm->getSpeckleRange());
        break;
    case SV::STEREO_MATCH::SGBM:
        match_param.push_back(sgbm->getPreFilterCap());
        match_param.push_back(sgbm->getBlockSize());
        match_param.push_back(sgbm->getMinDisparity());
        match_param.push_back(sgbm->getNumDisparities());
        match_param.push_back(sgbm->getUniquenessRatio());
        match_param.push_back(sgbm->getSpeckleWindowSize());
        match_param.push_back(sgbm->getSpeckleRange());
        break;
    }
    if (!match_param.empty())
        emit sendCurrentParams(match_param);
}

void stereo_vision::change_bm_pre_filter_size(int value)
{
    bm->setPreFilterSize(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getPreFilterSize();
#endif
}

void stereo_vision::change_bm_pre_filter_cap(int value)
{
    bm->setPreFilterCap(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getPreFilterCap();
#endif
}

void stereo_vision::change_bm_sad_window_size(int value)
{
    bm->setBlockSize(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getBlockSize();
#endif
}

void stereo_vision::change_bm_min_disp(int value)
{
    bm->setMinDisparity(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getMinDisparity();
#endif
}

void stereo_vision::change_bm_num_of_disp(int value)
{
    bm->setNumDisparities(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getNumDisparities();
#endif
}

void stereo_vision::change_bm_texture_thresh(int value)
{
    bm->setTextureThreshold(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getTextureThreshold();
#endif
}

void stereo_vision::change_bm_uniqueness_ratio(int value)
{
    bm->setUniquenessRatio(value);
    qDebug()<<bm->getUniquenessRatio();
}

void stereo_vision::change_bm_speckle_window_size(int value)
{
    bm->setSpeckleWindowSize(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getSpeckleWindowSize();
#endif
}

void stereo_vision::change_bm_speckle_range(int value)
{
    bm->setSpeckleRange(value);
#ifdef debug_info_sv_param
    qDebug()<<bm->getSpeckleRange();
#endif
}

void stereo_vision::change_sgbm_pre_filter_cap(int value)
{
    sgbm->setPreFilterCap(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getPreFilterCap();
#endif
}

void stereo_vision::change_sgbm_sad_window_size(int value)
{
    sgbm->setBlockSize(value);
    sgbm->setP1(8 * cn * value * value);
    sgbm->setP2(32 * cn * value * value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getBlockSize();
#endif
}

void stereo_vision::change_sgbm_min_disp(int value)
{
    sgbm->setMinDisparity(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getMinDisparity();
#endif
}

void stereo_vision::change_sgbm_num_of_disp(int value)
{
    sgbm->setNumDisparities(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getNumDisparities();
#endif
}

void stereo_vision::change_sgbm_uniqueness_ratio(int value)
{
    sgbm->setUniquenessRatio(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getUniquenessRatio();
#endif
}

void stereo_vision::change_sgbm_speckle_window_size(int value)
{
    sgbm->setSpeckleWindowSize(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getSpeckleWindowSize();
#endif
}

void stereo_vision::change_sgbm_speckle_range(int value)
{
    sgbm->setSpeckleRange(value);
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getSpeckleRange();
#endif
}
