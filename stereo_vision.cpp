#include "stereo_vision.h"

stereo_vision::stereo_vision() : TopView(20, 200, 3000, 19.8, 1080, 750, 270, 125, 100)
{
    device_index_L = -1;
    device_index_R = -1;
    fg_cam_L = false;
    fg_cam_R = false;
    fg_cam_opened = false;
    fg_calib_loaded = false;
    fg_calib = false;
    fg_stereoMatch = false;
    fg_reproject = false;
    fg_topview_plot_points = false;

    // blob
    obj_nums = 256;

    // thickness of frawing paint for objects on the image
    thick_obj_rect = 2;
    radius_obj_point = 3;

    objects = new objInformation[obj_nums];

    data = new StereoData * [IMG_H];
    for (int r = 0; r < IMG_H; r++) {
        data[r] = new StereoData[IMG_W];
    }

    // input source
    input_mode = SV::INPUT_SOURCE::CAM;

    // Initialization images for displaying
    img_r_L = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    img_r_R = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    disp = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);
    disp_pseudo = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    bm = cv::createStereoBM(16, 9);
    sgbm = cv::createStereoSGBM(0, 16, 3);
    img_detected = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);

    match_mode = -1;
    matchParamInitialize(SV::STEREO_MATCH::SGBM);

    time_gap = 10;
    t.start();
}

stereo_vision::~stereo_vision()
{
    for (int i = 0; i < IMG_H; i++)
        delete[] data[i];
    delete[] data;

    delete[] objects;

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
    if (fg_calib_loaded && this->cam_param.cam_focal_length == cam_focal_length && this->cam_param.base_line == base_line)
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
    this->cam_param.cam_focal_length = cam_focal_length;
    this->cam_param.base_line = base_line;
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

    // depth calculation of points from disp [merge into stereo_vision::depthCalculation]
}


void stereo_vision::depthCalculation()
{
    if (fg_pseudo)
        disp_pseudo.setTo(0);
    uchar* ptr_color = color_table->scanLine(0);
    for (int r = 0; r < IMG_H; r++) {
        short int* ptr_raw = (short int*) (disp_raw.data + r * disp_raw.step);
        uchar* ptr = (uchar*) (disp_pseudo.data + r * disp_pseudo.step);
        for (int c = 0; c < IMG_W; c++) {
            // non-overlapping part
            if (c < param_sgbm.num_of_disp / 2 && input_mode == SV::STEREO_MATCH::SGBM)
                continue;
            else if (c < param_bm.num_of_disp / 2 && input_mode == SV::STEREO_MATCH::BM)
                continue;
            // Depth calculation
            lock.lockForWrite();
            data[r][c].disp = ptr_raw[c];
            lock.unlock();
            if (data[r][c].disp > 0) {
                lock.lockForWrite();
                data[r][c].Z = cam_param.param_r / ptr_raw[c];
                lock.unlock();

                // pseudo color transform
                if (fg_pseudo) {
                    int z_est;
                    lock.lockForRead();
                    z_est = data[r][c].Z;
                    lock.unlock();
                    //                        std::cout<<z_est<<" ";
                    if (z_est >= min_distance && z_est <= max_distance) {
                        int jj = z_est - min_distance;
                        ptr[3 * c + 0] = ptr_color[3 * jj + 0];
                        ptr[3 * c + 1] = ptr_color[3 * jj + 1];
                        ptr[3 * c + 2] = ptr_color[3 * jj + 2];
                    }
                    else if (z_est > max_distance) {
                        ptr[3 * c + 0] = 0;
                        ptr[3 * c + 1] = 0;
                        ptr[3 * c + 2] = 255;
                    }
                    else {
                        ptr[3 * c + 0] = 0;
                        ptr[3 * c + 1] = 0;
                        ptr[3 * c + 2] = 0;
                    }
                }
            }
            else {
                lock.lockForRead();
                data[r][c].Z = -1;
                lock.unlock();
                //                    std::cout<<"0 ";
            }
        }
        //            std::cout<<std::endl;
    }
}

bool stereo_vision::stereoVision()
{
#ifdef debug_info_sv
    qDebug()<<"run";
#endif

    // camera capturing
    switch (input_mode) {
    case SV::INPUT_SOURCE::CAM:
        camCapture();
        break;
    case SV::INPUT_SOURCE::IMG:
//        img_L = ;
//        img_R = ;
        break;
    }

    // camera calibration
    if (fg_calib) {
        if (!rectifyImage())
            return false;
    }
    else {
        img_r_L = img_L.clone();
        img_r_R = img_R.clone();
    }

    detected_obj = 0;

    // stereo matching
    if (fg_stereoMatch) {
        stereoMatch();
        depthCalculation();

        if (fg_topview) {
            pointProjectTopView();

            if (fg_reproject)
                pointProjectImage();
        }
    }
    else {
        disp.setTo(0);
        disp_pseudo.setTo(0);
        img_detected.setTo(0);
    }

#ifdef debug_info_sv
    qDebug()<<"run"<<&img_L<<"emit"<<&img_r_L;
#endif

    if (t.elapsed() > time_gap) {
        emit svUpdateGUI(&img_r_L, &img_r_R, &disp, &disp_pseudo, &topview, &img_detected, detected_obj);
        t.restart();
    }
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

void stereo_vision::pointProjectTopView()
{
    resetTopView();
    int grid_row, grid_col;
    for (int r = 0; r < IMG_H; r++) {
        for (int c = 0; c < IMG_W; c++) {
            // reset
            data[r][c].grid_id = std::pair<int, int>(-1, -1);

            // porject each 3D point onto a topview
            if (data[r][c].disp > 0) {
                grid_row = 1.0 * log10(1.0 * data[r][c].Z / min_distance) / log10(1.0 + k);
//                grid_col = 360.0 * img_col_half * atan((c / (double)(IMG_W / img_col) - img_col_half) / data[r][c].Z) / (view_angle * CV_PI) + img_col_half;
//                grid_col = 1.0 * c * ratio_col; //**// old
                grid_col = 1.0 * c / this->c;

                // display each point on topview
                if (fg_topview_plot_points)
                    if (grid_row >= 0 && grid_row < img_row + 1 &&
                            grid_col >= 0 && grid_col < img_col + 1)
                        cv::circle(topview, pointT(img_grid[grid_row][grid_col]), 1, cv::Scalar(0, 0, 255, 255), 1, 8, 0);

                // mark each point belongs to which cell
                int grid_row_t = img_row - grid_row - 1;
                int grid_col_t = grid_col;
                if (grid_row_t >= 0 && grid_row_t < img_row &&
                        grid_col_t >= 0 && grid_col_t < img_col) {
                    lock.lockForWrite();
                    // count the amount of point
                    grid_map[grid_row_t][grid_col_t].pts_num++;
                    // average the depth
                    grid_map[grid_row_t][grid_col_t].avg_Z += 1.0 * (data[r][c].Z - grid_map[grid_row_t][grid_col_t].avg_Z) / grid_map[grid_row_t][grid_col_t].pts_num;
                    // label the point to the belonging cell
                    data[r][c].grid_id = std::pair<int, int>(grid_row_t, grid_col_t);
                    lock.unlock();
                }
            }
        }
    }

    blob(3000);

    // check whether the cell is satisfied as an object [merge into stereo_vision::blob]
}

void stereo_vision::resetBlob()
{
    for (int i = 0; i < obj_nums; i++) {
        objects[i].labeled = false;
        objects[i].tl = std::pair<int, int>(-1, -1);
        objects[i].br = std::pair<int, int>(-1, -1);
        objects[i].center = std::pair<int, int>(-1, -1);
        objects[i].avg_Z = 0;
        objects[i].pts_num = 0;
        objects[i].closest_count = 0;
    }
}

void stereo_vision::blob(int thresh_pts_num)
{
    // objects starts from 0
    int mask_size = 3;
    int offset = (mask_size - 1) / 2;
    int cur_label = 0;
    resetBlob();
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            // blob labeling
            if (grid_map[r][c].pts_num >= thresh_free_space && grid_map[r][c].obj_label == -1) {
                if (cur_label == obj_nums) {
                    qDebug()<<"Objects are over defined.";
                    break;
                }

                int count = 1;

                std::stack<std::pair<int, int> > neighbors;
                neighbors.push(std::pair<int, int>(r, c));
                lock.lockForWrite();
                grid_map[r][c].obj_label = cur_label;
                objects[cur_label].pts_num += grid_map[r][c].pts_num;
                objects[cur_label].avg_Z += 1.0 * (grid_map[r][c].avg_Z - objects[cur_label].avg_Z) / count;
                lock.unlock();

                while (!neighbors.empty()) {
                    std::pair<int, int> cur_pos = neighbors.top();
                    neighbors.pop();

                    int r_now, c_now;
                    for (int rr = - offset; rr <= offset; rr++) {
                        r_now = cur_pos.first + rr;
                        for (int cc = - offset; cc <= offset; cc++) {
                            c_now = cur_pos.second + cc;

                            // out of boundary
                            if (r_now < 0 || r_now >= img_row ||
                                    c_now < 0 || c_now >= img_col) {
                                continue;
                            }
                            if (grid_map[r_now][c_now].pts_num >= thresh_free_space &&
                                    grid_map[r_now][c_now].obj_label == -1) {
                                neighbors.push(std::pair<int, int>(r_now, c_now));
                                lock.lockForWrite();
                                grid_map[r_now][c_now].obj_label = cur_label;
                                objects[cur_label].pts_num += grid_map[r_now][c_now].pts_num;
                                objects[cur_label].avg_Z += 1.0 * (grid_map[r][c].avg_Z - objects[cur_label].avg_Z) / count;
                                lock.unlock();
                            }
                        }
                    }

                }
                cur_label++;
                count++;
            }
        }
        if (cur_label == obj_nums)
            break;
    }

    for (int i= 0; i < obj_nums; i++) {
        if (objects[i].pts_num >= thresh_pts_num) {
            objects[i].labeled = true;
            detected_obj++;
        }
    }

#ifdef debug_info_sv_blob_data
    std::cout<<"objs: "<<cur_label - 1<<std::endl;

    for (int i = 0; i < obj_nums; i++) {
        std::cout<<i<<" "<<objects[i].pts_num<<std::endl;
    }

    // blob data of grid map
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            std::cout << grid_map[r][c].obj_label << " ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
#endif

    // filter out trivial objects by amount of points & check whether the cell is satisfied as an object
    cv::Point pts[4];
    uchar* ptr_color = color_table->scanLine(0);
    int p;
    int gap = 0;
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            // filtering
            if (grid_map[r][c].obj_label >= 0 && objects[grid_map[r][c].obj_label].labeled) {
                // plot points onto topview
                int row = img_row - r - gap > 0 ? img_row - r - gap : 0;
                int row_1 = img_row - (r + 1) + gap <= img_row ? img_row - (r + 1) + gap : img_row;
                int col = c - gap > 0 ? c - gap : 0;
                int col_1 = c + 1 + gap <= img_col ? c + 1 + gap : img_col;

                pts[0] = pointT(img_grid[row][col]);
                pts[1] = pointT(img_grid[row_1][col]);
                pts[2] = pointT(img_grid[row_1][col_1]);
                pts[3] = pointT(img_grid[row][col_1]);

//                p = 256.0 / (1.0 * obj_nums) * grid_map[r][c].obj_label * (max_distance - min_distance);
                p = (max_distance - 0.5 * (img_grid[row][col].y + img_grid[row_1][col].y)) - min_distance;

                cv::fillConvexPoly(topview, pts, thick_polygon, cv::Scalar(ptr_color[3 * p + 0], ptr_color[3 * p + 1], ptr_color[3 * p + 2], 255), 8, 0);
            }
        }
    }

    // find the boundary of objects [merge into stereo_vision::pointProjectImage]
}

void stereo_vision::pointProjectImage()
{
    // remap a blob objects from topview to label
    lock.lockForWrite();
    img_detected.setTo(0);
    lock.unlock();
    for (int r = 0; r < IMG_H; r++) {
        uchar *ptr_o = img_r_L.ptr<uchar>(r);
        uchar *ptr_d = img_detected.ptr<uchar>(r);
        for (int c = 0; c < IMG_W; c++) {
            int grid_r, grid_c;
            lock.lockForRead();
            grid_r = data[r][c].grid_id.first;
            grid_c = data[r][c].grid_id.second;
            lock.unlock();
            if (grid_r == -1 || grid_c == -1 || data[r][c].disp <= 0)
                continue;
            int label = grid_map[grid_r][grid_c].obj_label;
            if (grid_map[grid_r][grid_c].obj_label >= 0 && objects[label].labeled && data[r][c].disp > 0) {
                // find the boundary of objects
                if (objects[label].br == std::pair<int, int>(-1, -1) &&
                        objects[label].tl == std::pair<int, int>(-1, -1)) {
                    objects[label].br = std::pair<int, int>(r, c);
                    objects[label].tl = std::pair<int, int>(r, c);
                }
                else {
                    lock.lockForWrite();
                    if (objects[label].br.first < r)
                        objects[label].br.first = r;
                    if (objects[label].br.second < c)
                        objects[label].br.second = c;
                    if (objects[label].tl.first > r)
                        objects[label].tl.first = r;
                    if (objects[label].tl.second > c)
                        objects[label].tl.second = c;
                    lock.unlock();
                }

                // draw points
                ptr_d[3 * c + 0] = ptr_o[3 * c + 0];
                ptr_d[3 * c + 1] = ptr_o[3 * c + 1];
                ptr_d[3 * c + 2] = ptr_o[3 * c + 2];
            }
        }
    }

    // sort objects using depth for displaying color
    for (int i = 0; i < obj_nums; i++) {
        for (int j = i + 1; j < obj_nums; j++) {
            if (objects[i].avg_Z >= objects[j].avg_Z) {
                obj_temp = objects[j];
                objects[j] = objects[i];
                objects[i] = obj_temp;
            }
        }
    }

    // draw region of detected objects
    for (int i = 0; i < obj_nums; i++) {
        uchar* ptr_color = color_table->scanLine(0);
        int tag = objects[i].avg_Z - min_distance;
        lock.lockForWrite();
        if (objects[i].labeled && objects[i].br != std::pair<int, int>(-1, -1) && objects[i].tl != std::pair<int, int>(-1, -1)) {
            // find center of rect
            objects[i].center = std::pair<int, int>(0.5 * (objects[i].tl.first + objects[i].br.first), 0.5 * (objects[i].tl.second + objects[i].br.second));

            cv::rectangle(img_detected, cv::Rect(objects[i].tl.second, objects[i].tl.first, objects[i].br.second - objects[i].tl.second, objects[i].br.first - objects[i].tl.first),
                          cv::Scalar(ptr_color[3 * tag + 0], ptr_color[3 * tag + 1], ptr_color[3 * tag + 2]), thick_obj_rect, 8, 0);
            cv::circle(img_detected, cv::Point(objects[i].center.second, objects[i].center.first), radius_obj_point, cv::Scalar(0, 255, 0), -1, 8, 0);
        }
        lock.unlock();
    }
}

