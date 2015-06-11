#include "sensorinfo.h"

SensorInfo::SensorInfo()
{
    //**// You may CRASH if you add lots of sensors but you don't revise the number of the array
    // sensor information
    sensors = new sensorInformation[3];

    // Initialization
    sv = new stereo_vision();
    rc = new RadarController();
    lrf = new lrf_controller();
    size_data_fused = sv->objSize() + rc->objSize();
    data_fused = new ObjectTracking::objectTrackingInfo[size_data_fused];

    fg_fusion = false;

    fg_ca_astar = false;

    ot_fused = new ObjectTracking();
    // max. size of detected objects
//    ot_fused->initialze(sv->objSize() + rc->objSize());

    // object macthing
    initializeObjectMatching(size_data_fused);
    setErrorThresholdX(300);
    setErrorThresholdZ(500, sv->max_distance);
    setThresholdBha(0.3);

    ca = new CollisionAvoidance();

    pic_sv = QPixmap(20, 20);
    pic_radar = QPixmap(20, 20);

    range_precision = 3;

    t.restart();
    time_gap = 30;

#ifdef debug_info_ca_astar
    cv::namedWindow("A* result", cv::WINDOW_AUTOSIZE);
#endif
}

SensorInfo::~SensorInfo()
{
    delete sv;
    delete rc;
    delete lrf;
    delete[] data_fused;
    delete ot_fused;
    delete ca;
    delete[] sensors;
}

void SensorInfo::initialFusedTopView(int range_pixel)
{
    // vehicle & fused topview information
    detection_range_pixel = range_pixel;

    detection_range = sv->max_distance;

    ratio = 1.0 * detection_range_pixel / sv->max_distance;

    // use cart as default
    vehicle.width = 43;
    vehicle.length = 43;
    vehicle.width_pixel = vehicle.width * ratio;
    vehicle.length_pixel = vehicle.length * ratio;

    vehicle.head_pos = vehicle.length / 2;

    // find max & min detection range in all sensors, so this function shall be called after initialization of all sensors
    max_detection_range = rc->max_distance;

    min_detection_range = rc->min_distance + sqrt(pow(0.5 * vehicle.length, 2) + pow(0.5 * vehicle.width, 2));

    fused_topview.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);
    fused_topview_BG.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);

    thickness = 2;
    font = cv::FONT_HERSHEY_PLAIN;
    font_size = 1;
    font_thickness = 1;

    // sv
    int sensor = SENSOR::SV;
    sensors[sensor].color = cv::Scalar(255, 0, 0, 255);

    sensors[sensor].angle_half_fov = sv->view_angle * 0.5 * CV_PI / 180.0;

    sensors[sensor].location.pos = cv::Point(0, 51);

    sensors[sensor].location.theta = 0.0;

    QColor pic_color_sv = QColor(sensors[sensor].color[0], sensors[sensor].color[1], sensors[sensor].color[2]);
    pic_sv.fill(pic_color_sv);

    // radar
    sensor = SENSOR::RADAR;
    sensors[sensor].color = cv::Scalar(0, 0, 255, 255);

    sensors[sensor].angle_half_fov = rc->view_angle * 0.5 * CV_PI / 180.0;

    sensors[sensor].location.pos = cv::Point(0, vehicle.head_pos);

    sensors[sensor].location.theta = 0.0;

    QColor pic_color_radar = QColor(sensors[sensor].color[0], sensors[sensor].color[1], sensors[sensor].color[2]);
    pic_radar.fill(pic_color_radar);
}

void SensorInfo::updateFusedTopView()
{
    ratio = 1.0 * detection_range_pixel / detection_range;

    vehicle.VCP = cv::Point(detection_range_pixel, detection_range_pixel);

    vehicle.rect = cv::Rect(vehicle.VCP.x - (int)(vehicle.width_pixel / 2.0), vehicle.VCP.y - (int)(vehicle.length_pixel / 2.0),
                            vehicle.width_pixel, vehicle.length_pixel);

    vehicle.color = cv::Scalar(0, 255, 0, 255);

    fused_topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    // topview
    cv::circle(fused_topview_BG, vehicle.VCP, detection_range_pixel, rc->color_BG, -1, 8, 0);

    // max detection range [now]
    cv::putText(fused_topview_BG, QString::number((int)detection_range).toStdString() + " cm", cv::Point(vehicle.VCP.x + 0.65 * detection_range_pixel, vehicle.VCP.y  - 0.8 * detection_range_pixel), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, rc->color_line, 1);

    // sensor information
    int device = SENSOR::SV;
    sensors[device].pos_pixel = cv::Point(sensors[device].location.pos.x * ratio, sensors[device].location.pos.y * ratio);
    device = SENSOR::RADAR;
    sensors[device].pos_pixel = cv::Point(sensors[device].location.pos.x * ratio, sensors[device].location.pos.y * ratio);

    // FOV - Stereo vision
    cv::Point sensor_pos;
    cv::Point pos_fov_r, pos_fov_l;
    int shift_x, shift_y;
    device = SENSOR::SV;
    sensor_pos = cv::Point(vehicle.VCP.x + sensors[device].pos_pixel.x, vehicle.VCP.y - sensors[device].pos_pixel.y);
    shift_x = detection_range_pixel * sin(sensors[device].angle_half_fov);
    shift_y = detection_range_pixel * cos(sensors[device].angle_half_fov);
    pos_fov_r = cv::Point(sensor_pos.x + shift_x, sensor_pos.y - shift_y);
    pos_fov_l = cv::Point(sensor_pos.x - shift_x, sensor_pos.y - shift_y);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_r, sensors[device].color_fov, 1, 8, 0);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_l, sensors[device].color_fov, 1, 8, 0);

    // FOV - ESR
    device = SENSOR::RADAR;
    sensor_pos = cv::Point(vehicle.VCP.x + sensors[device].pos_pixel.x, vehicle.VCP.y - sensors[device].pos_pixel.y);
    shift_x = detection_range_pixel * sin(sensors[device].angle_half_fov);
    shift_y = detection_range_pixel * cos(sensors[device].angle_half_fov);
    pos_fov_r = cv::Point(sensor_pos.x + shift_x, sensor_pos.y - shift_y);
    pos_fov_l = cv::Point(sensor_pos.x - shift_x, sensor_pos.y - shift_y);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_r, sensors[device].color_fov, 1, 8, 0);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_l, sensors[device].color_fov, 1, 8, 0);

    // 30 m
    float distance = sv->max_distance * ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // 5 m
    distance = 500 * ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // vehicle
    cv::rectangle(fused_topview_BG, vehicle.rect, vehicle.color, -1, 8, 0);
}

void SensorInfo::chooseVehicle(int vehicle_type)
{
    switch (vehicle_type) {
    case VEHICLE::CART:
        // choose cart
        vehicle.width = 43;
        vehicle.length = 43;
        vehicle.head_pos = vehicle.length / 2;

        sensors[SENSOR::SV].location.pos = cv::Point(0, 29.5);
        sensors[SENSOR::RADAR].location.pos = cv::Point(0, vehicle.head_pos);
        break;
    case VEHICLE::CAR:
        // choose car: TOYOTA Camry
        vehicle.width = 183;
        vehicle.length = 485;
        vehicle.head_pos = vehicle.length / 2;

        sensors[SENSOR::SV].location.pos = cv::Point(0, 75);
        sensors[SENSOR::RADAR].location.pos = cv::Point(0, vehicle.head_pos);
        break;
    }

    vehicle.width_pixel = vehicle.width * ratio;
    vehicle.length_pixel = vehicle.length * ratio;
    // find max & min detection range in all sensors, so this function shall be called after initialization of all sensors
    max_detection_range = rc->max_distance;
    min_detection_range = rc->min_distance + sqrt(pow(0.5 * vehicle.length, 2) + pow(0.5 * vehicle.width, 2));

    updateFusedTopView();
}

void SensorInfo::zoomOutFusedTopView()
{
    // zoom out (scrolling forward)
    if (detection_range == min_detection_range) {
        int count = min_detection_range / gap;
        if (count == 0)
            count = 1;
        detection_range = ceil(count) * gap;
    }
    else
        detection_range = detection_range + gap < max_detection_range ?
                    detection_range + gap : max_detection_range;
}

void SensorInfo::zoomInFusedTopView()
{
    // zoom in (scrolling backward)
    if (detection_range == max_detection_range) {
        int count = max_detection_range / gap;
        detection_range = floor(count) * gap;
    }
    else
        detection_range = detection_range - gap > min_detection_range ?
                    detection_range - gap : min_detection_range;
}

void SensorInfo::resetFusion()
{
    for (int p = 0; p < dataFusedSize(); p++)
        resetObjectTrackingInfo(data_fused[p]);
}

void SensorInfo::dataExec(bool fg_sv, bool fg_radar, bool fg_data_update, bool fg_fusion, bool fg_om, bool fg_ot_kf, bool fg_ca_astar, bool fg_sv_each)
{
    if (fg_data_update) {
        resetFusion();

        this->fg_fusion = fg_fusion && (fg_sv && fg_radar);
        this->fg_ca_astar = fg_ca_astar && (fg_sv || fg_radar);

        dataProcess(fg_sv, fg_radar);

        if (fg_om) {
            dataMatching();

            if (fg_ot_kf)
                dataTracking();
        }

        dataCollisionAvoidance();
    }

    drawFusedTopView(fg_sv, fg_radar, fg_sv_each);
}

void SensorInfo::resetObjectTrackingInfo(ObjectTracking::objectTrackingInfo &src)
{
    src.det_mode = DETECT_MODE::NO_DETECT;
    src.id.x = -1;
    src.id.y = -1;
    src.id.z = -1;
    src.img.release();
    src.pc = SensorBase::PC(0.0, 0.0);
    src.pos = cv::Point(-1, -1);
    src.vel = cv::Point2f(0, 0);
    src.acc = cv::Point2f(0, 0);
}

void SensorInfo::moveTrackingInfo(ObjectTracking::objectTrackingInfo &src, ObjectTracking::objectTrackingInfo &dst)
{
    dst.det_mode    = src.det_mode;
    dst.id          = src.id;
    dst.img.release();
    dst.img         = src.img.clone();
    dst.rect_f      = src.rect_f;
    dst.plot_pt_f   = src.plot_pt_f;
    dst.pc          = src.pc;
    dst.pos         = src.pos;
    dst.rect        = src.rect;
    dst.vel         = src.vel;
    dst.acc         = src.acc;

    resetObjectTrackingInfo(src);
}

void SensorInfo::dataMatching()
{
    // Object matching: a) Bha. dist. of H color space, b) bias of X & Z of WCS location
    // Comparison of H color space image using Bhattacharyya distance with Bubble search
    resetObjMatching();
    // Extract histogram of H color space
    lock_data_fused.lockForRead();
    for (int i = 0; i < size_data_fused; i++) {
        if (data_fused[i].det_mode != DETECT_MODE::NO_DETECT) {
            fg_om_existed = true;
            om[i].labeled = true;
            om_obj_num++;
            om[i].pc = data_fused[i].pc;
            om[i].center = std::pair<int, int>(0, 0);
            switch (data_fused[i].det_mode) {
            case DETECT_MODE::RADAR_ONLY:
                om[i].match_type = MATCH_TYPE::RANGE_ONLY;
                break;
            case DETECT_MODE::SV_ONLY:
            case DETECT_MODE::SV_RADAR:
                om[i].match_type = MATCH_TYPE::RANGE_BHA;
                cv::Mat img_hsv = cv::Mat(data_fused[i].img.rows, data_fused[i].img.cols, CV_8UC3);
                om[i].img = data_fused[i].img.clone();
                cv::cvtColor(data_fused[i].img, img_hsv, cv::COLOR_BGR2HSV);
                splitOneOut(0, img_hsv, &om[i].H_img);
                cv::calcHist(&om[i].H_img, 1, 0, cv::Mat(), om[i].H_hist, 1, &hist_size, &hist_ranges, true, false);
                cv::normalize(om[i].H_hist, om[i].H_hist, hist_ranges[0], hist_ranges[1], cv::NORM_MINMAX, -1, cv::Mat());
                break;
            }
        }
    }
    lock_data_fused.unlock();

#ifdef debug_info_object_matching_img
    comp = img_detected.clone();
#endif

    matching_result.clear();
    matching_result = Matching();

    for (int i = 0; i < matching_result.size(); i++) {
        moveTrackingInfo(data_fused[matching_result[i].first], data_fused[matching_result[i].second]);
    }
}

void SensorInfo::dataTracking()
{
    for (int m = 0; m < ot_fused->ti.size(); m++)
        ot_fused->ti[m].fg_update = false;

    for (int p = 0 ; p < size_data_fused; p++) {
        if (data_fused[p].det_mode == DETECT_MODE::NO_DETECT) continue;
        bool fg_new_obj = true;
        int conti_id = -1;
        for (int m = 0; m < ot_fused->ti.size(); m++) {
            if (ot_fused->ti[m].track_status == TRACK_STATUS::NO_TARGET || ot_fused->ti[m].fg_update)
                continue;

            int cur_obj = ot_fused->ti[m].info.size() - 1;
            lock_data_fused.lockForRead();
            if (ot_fused->ti[m].info[cur_obj].id.y == data_fused[p].id.y ||
                    ot_fused->ti[m].info[cur_obj].id.z == data_fused[p].id.z) {
                fg_new_obj = false;
                conti_id = m;
                break;
            }
            lock_data_fused.unlock();
        }

        lock_data_fused.lockForRead();
        ObjectTracking::objectTrackingInfo info_new;
        info_new.id.x = conti_id;
        info_new.id.y = data_fused[p].id.y;
        info_new.id.z = data_fused[p].id.z;
        info_new.det_mode = data_fused[p].det_mode;
        info_new.img = data_fused[p].img.clone();
        info_new.rect_f = data_fused[p].rect_f;
        info_new.plot_pt_f = data_fused[p].plot_pt_f;
        info_new.pc = data_fused[p].pc;
        info_new.pos = data_fused[p].pos;
        info_new.vel = data_fused[p].vel;
        info_new.acc = data_fused[p].acc;
        info_new.rect = data_fused[p].rect;
        lock_data_fused.unlock();
        // update object
        if (!fg_new_obj) {
            ot_fused->ti[conti_id].info.push_back(info_new);
            ot_fused->ti[conti_id].track_status = TRACK_STATUS::UPDATE_TARGET;
            ot_fused->ti[conti_id].fg_update = true;

            // Kalman filter
            int last = ot_fused->ti[conti_id].info.size() - 1;
            if (last > 2) {
                // (m), (m/s)
                cv::Point2f pt_pre2t = cv::Point2f(ot_fused->ti[conti_id].info[last - 2].pos.x / 100.0,
                        ot_fused->ti[conti_id].info[last - 2].pos.y / 100.0);
                cv::Point2f pt_pre1t = cv::Point2f(ot_fused->ti[conti_id].info[last - 1].pos.x / 100.0,
                        ot_fused->ti[conti_id].info[last - 1].pos.y / 100.0);
                cv::Point2f vel = ot_fused->ti[conti_id].info[last - 1].vel;
                std::cout<<"pt_pre1t: "<<pt_pre1t.x<<","<<pt_pre1t.y<<"\t"<<
                                         pt_pre2t.x<<","<<pt_pre2t.y<<"\t"<<
                                         vel.x     <<","<<vel.y     <<std::endl;
                ot_fused->ti[conti_id].kf.kf_core.statePost.at<float>(0) = pt_pre2t.x + vel.x;
                ot_fused->ti[conti_id].kf.kf_core.statePost.at<float>(1) = pt_pre2t.y + vel.y;
                ot_fused->ti[conti_id].kf.statePt = cv::Point((int)(ot_fused->ti[conti_id].kf.kf_core.statePost.at<float>(0)), (int)(ot_fused->ti[conti_id].kf.kf_core.statePost.at<float>(1)));
                std::cout<<"statePt: "<<ot_fused->ti[conti_id].kf.statePt.x<<","<<ot_fused->ti[conti_id].kf.statePt.y<<std::endl;

                ot_fused->ti[conti_id].kf.prediction = ot_fused->ti[conti_id].kf.kf_core.predict();
                ot_fused->ti[conti_id].kf.predictPt = cv::Point((int)(ot_fused->ti[conti_id].kf.prediction.at<float>(0)), (int)(ot_fused->ti[conti_id].kf.prediction.at<float>(1)));
                std::cout<<"predictPt: "<<ot_fused->ti[conti_id].kf.predictPt.x<<","<<ot_fused->ti[conti_id].kf.predictPt.y<<std::endl;

                ot_fused->ti[conti_id].kf.measurement.at<float>(0) = pt_pre1t.x;
                ot_fused->ti[conti_id].kf.measurement.at<float>(1) = pt_pre1t.y;

                ot_fused->ti[conti_id].kf.kf_core.correct(ot_fused->ti[conti_id].kf.measurement);

                ot_fused->ti[conti_id].trajectory.push_back(point2FusedTopView(SensorBase::cart2Polar(ot_fused->ti[conti_id].kf.predictPt)));

                int last_traj = ot_fused->ti[conti_id].trajectory.size() - 1;
                cv::Point traj_prev1 = ot_fused->ti[conti_id].trajectory[last_traj - 1];
                cv::Point traj_now = ot_fused->ti[conti_id].trajectory[last_traj];
                drawArrow(fused_topview, traj_prev1, traj_now, 5, 30, cv::Scalar(255, 255, 0, 255), 1, 8);
                std::cout<<"Trajectory\n";
                std::cout<<conti_id<<" "<<traj_prev1.x<<","<<traj_prev1.y<<"\t"<<
                           traj_now.x<<","<<traj_now.y<<std::endl;
            }

#ifdef debug_info_fusion
            int id_need = -1;
            for (int k = 0; k < ot_fused->ti[conti_id].info.size(); k++) {
                if (ot_fused->ti[conti_id].info[k].id.y != -1) {
                    id_need = k;
                    break;
                }
            }
            if (id_need != -1 && ot_fused->ti[conti_id].info[id_need].id.y != -1 &&
                    ot_fused->ti[conti_id].info[ot_fused->ti[conti_id].info.size() - 1].id.y != -1) {
                cv::Mat img_1 = ot_fused->ti[conti_id].info[id_need].img;
                cv::Mat img_2 = ot_fused->ti[conti_id].info[ot_fused->ti[conti_id].info.size() - 1].img;

                cv::namedWindow("first", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("now", cv::WINDOW_AUTOSIZE);
                cv::imshow("first", img_1);
                cv::imshow("now", img_2);
                char c;
                cv::waitKey(c);
                cv::destroyWindow("first");
                cv::destroyWindow("now");
            }
            qDebug()<<"update"<<info_new.id.x<<info_new.id.y<<info_new.id.z;
#endif
        }
        // new object
        else {
            ObjectTracking::objectTracking ti_new;
            info_new.id.x = ti_new.info.size();
            ti_new.info.push_back(info_new);
            ti_new.track_status = TRACK_STATUS::NEW_TARGET;
            ti_new.fg_update = true;
            cv::setIdentity(ti_new.kf.kf_core.measurementMatrix);
            cv::setIdentity(ti_new.kf.kf_core.processNoiseCov, cv::Scalar::all(1e-5));
            cv::setIdentity(ti_new.kf.kf_core.measurementNoiseCov, cv::Scalar::all(1e-1));
            cv::setIdentity(ti_new.kf.kf_core.errorCovPost, cv::Scalar::all(1));
            ot_fused->ti.push_back(ti_new);
#ifdef debug_info_fusion
            qDebug()<<"new"<<info_new.id.x<<info_new.id.y<<info_new.id.z;
#endif
        }
    }

    for (int m = 0; m < ot_fused->ti.size(); m++) {
        if (!ot_fused->ti[m].fg_update) {
            ot_fused->ti[m].missed_count++;
            if (ot_fused->ti[m].missed_count == ot_fused->thresh_count)
                ot_fused->resetObjectTracking(ot_fused->ti[m]);
        }
        else
            ot_fused->ti[m].missed_count = 0;
    }
}

void SensorInfo::dataCollisionAvoidance()
{
    if (!fg_ca_astar)
        return;

    // reset A* map
    ca->astar->resetMap();

    lock_data_fused.lockForWrite();
    for (int p = 0 ; p < size_data_fused; p++) {
        if (data_fused[p].det_mode == DETECT_MODE::SV_ONLY ||
                data_fused[p].det_mode == DETECT_MODE::SV_RADAR) {
            cv::Point tl, br;
            tl = cv::Point(data_fused[p].rect_f.tl().x, data_fused[p].rect_f.tl().y);
            br = cv::Point(data_fused[p].rect_f.br().x, data_fused[p].rect_f.br().y);
            for (int c = tl.x; c <= br.x; c++) {
                for (int r = tl.y; r <= br.y; r++) {
                    ca->astar->kernelDilation(cv::Point(c, r),
                                              cv::Size(vehicle.width_pixel, vehicle.length_pixel));
                }
            }
        }
        else if (data_fused[p].det_mode == DETECT_MODE::RADAR_ONLY) {
            ca->astar->kernelDilation(data_fused[p].plot_pt_f, cv::Size(vehicle.width_pixel + 9, vehicle.length_pixel + 11));
        }
    }
    lock_data_fused.unlock();

    // A* path planning
    ca->path = ca->astar->PathFind2D(320, 320, 320, 0);

#ifdef debug_info_ca_astar
    int ratio = 1;
    int display_w = MAP_WIDTH * ratio;
    int display_h = MAP_HEIGHT * ratio;
    cv::Mat img_result = cv::Mat(display_h, display_w, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int c = 0; c < MAP_WIDTH; c++) {
        for (int r = 0; r < MAP_HEIGHT; r++) {
            if (ca->astar->map[c][r] == 1) {
                for (int rr = r * ratio; rr < (r + 1) * ratio; rr++) {
                    for (int cc = c * ratio; cc < (c + 1) * ratio; cc++) {
                        img_result.at<cv::Vec3b>(rr, cc) = cv::Vec3b(0, 0, 0);
                    }
                }
            }
        }
    }

    if (!ca->path.empty()) {
        for (int k = 0; k < ca->path.size(); k++) {
            int map_x = ca->path[k].first;
            int map_y = ca->path[k].second;
            for (int r = map_y * ratio; r < (map_y + 1) * ratio; r++) {
                for (int c = map_x * ratio; c < (map_x + 1) * ratio; c++) {
                    img_result.at<cv::Vec3b>(r, c) = cv::Vec3b(128, 128, 128);
                }
            }
        }
    }

    cv::imshow("A* result", img_result);
//    cv::imwrite("Astar.jpg", img_result);
#endif
}

void SensorInfo::dataProcess(bool fg_sv, bool fg_radar)
{
    stereo_vision::objectInfo *d_sv = sv->objects_display;
    RadarController::objectTrackingInfo *d_radar = rc->objects_display;

    // Stereo vision
    int device = SENSOR::SV;
    if (fg_sv) {
        for (int k = 0; k < sv->objSize(); k++) {
            if (d_sv[k].labeled) {
                d_sv[k].pc_world = coordinateTransform(CT_SCS2WCS, sensors[device].pos_pixel, d_sv[k].pc);
                d_sv[k].plot_pt_f = point2FusedTopView(sensors[device].pos_pixel, d_sv[k].pc);
                d_sv[k].rect_f = rectOnFusedTopView(d_sv[k].plot_pt_f, d_sv[k].rect);
            }
        }
    }
    // RADAR
    device = SENSOR::RADAR;
    if (fg_radar) {
        for (int m = 0; m < rc->objSize(); m++) {
            if (d_radar[m].status >= rc->obj_status_filtered) {
                d_radar[m].pc = SensorBase::PC(100 * d_radar[m].range, d_radar[m].angle + sensors[device].location.theta);
                d_radar[m].plot_pt_f = point2FusedTopView(sensors[device].pos_pixel, d_radar[m].pc);
                d_radar[m].pc_world = coordinateTransform(CT_SCS2WCS, sensors[device].pos_pixel, d_radar[m].pc);
            }
        }
    }

    // Data fusion & transform data type for later process
    int count = 0;
    for (int m = 0; m < rc->objSize(); m++)
        d_radar[m].fg_fused = false;

    if (fg_sv) {
        for (int k = 0; k < sv->objSize(); k++) {
            if (!d_sv[k].labeled) continue;

            closest_radar_id = -1;
            if (fg_fusion) {
                U_D = 500;    // max distance error (cm)
                R_sv = U_D * d_sv[k].pc_world.range / (1.0 * sv->max_distance);  // (cm)
                closest_radar_distance = 10000000.0;
                sv_pos = SensorBase::polar2Cart(d_sv[k].pc_world);
                //    std::cout<<"SV: "<<sv_pos.x<<" "<<sv_pos.y<<std::endl;
                for (int m = 0; m < rc->objSize(); m++) {
                    if (d_radar[m].status >= rc->obj_status_filtered && !d_radar[m].fg_fused) {
                        cv::Point2d radar_pos = SensorBase::polar2Cart(d_radar[m].pc);
                        double deviation = sqrt(pow((double)(radar_pos.x - sv_pos.x), 2) + pow((double)(radar_pos.y - sv_pos.y), 2));
                        //                std::cout<<deviation<<" "<<closest_radar_distance<<" "<<R_sv<<std::endl;
                        //                std::cout<<"RADAR: "<<radar.x<<" "<<radar.y<<std::endl;
                        if (deviation < closest_radar_distance && deviation < R_sv) {
                            closest_radar_id = m;
                            closest_radar_distance = deviation;
                            radar_pos_closest.x = radar_pos.x;
                            radar_pos_closest.y = radar_pos.y;
                        }
                        //                std::cout<<"dist. "<<sv_pos.x<<" "<<sv_pos.y<<"\t"<<radar_pos.x<<" "<<radar_pos.y<<std::endl;
                    }
                }
            }

            // un-fused - stereo vision
            if (closest_radar_id == -1) {
                data_fused[count].det_mode = DETECT_MODE::SV_ONLY;
                data_fused[count].id.x = count;
                data_fused[count].id.y = k;
                data_fused[count].img = d_sv[k].img.clone();
                data_fused[count].rect_f = d_sv[k].rect_f;
                data_fused[count].plot_pt_f = d_sv[k].plot_pt_f;
                data_fused[count].pc = d_sv[k].pc_world;
                data_fused[count].pos = SensorBase::polar2Cart(d_sv[k].pc_world);
                data_fused[count].vel = d_sv[k].vel;
                data_fused[count].rect = d_sv[k].rect_world;
                count++;
            }
            // fusion data
            else if (fg_fusion && closest_radar_id != -1){
                float ratio_radar = 0.9;
                float ratio_sv = 0.1;
                data_fused[count].det_mode = DETECT_MODE::SV_RADAR;
                data_fused[count].id.x = count;
                data_fused[count].id.y = k;
                data_fused[count].id.z = closest_radar_id;
                data_fused[count].img = d_sv[k].img.clone();
                data_fused[count].pc = SensorBase::PC(ratio_sv * d_sv[k].pc.range + ratio_radar * d_radar[closest_radar_id].pc.range,
                                                      ratio_sv * d_sv[k].pc.angle + ratio_radar * d_radar[closest_radar_id].pc.angle);
                data_fused[count].pos = SensorBase::polar2Cart(data_fused[count].pc);
                data_fused[count].plot_pt_f = ratio_sv * d_sv[k].plot_pt_f + ratio_radar * d_radar[closest_radar_id].plot_pt_f;
                cv::Point tl_new = cv::Point(data_fused[count].plot_pt_f.x - 0.5 * d_sv[k].rect_f.width, data_fused[count].plot_pt_f.y - 0.5 * d_sv[k].rect_f.height);
                data_fused[count].rect_f = cv::Rect(tl_new.x, tl_new.y, d_sv[k].rect_f.width, d_sv[k].rect_f.height);
                data_fused[count].vel = ratio_sv * d_sv[k].vel + ratio_radar * d_radar[closest_radar_id].vel;
                data_fused[count].rect = d_sv[k].rect_world;
                d_radar[closest_radar_id].fg_fused = true;
                count++;
            }
        }
    }

    // un-fused - radar
    if (fg_radar) {
        for (int m = 0; m < rc->objSize(); m++) {
            if (d_radar[m].status >= rc->obj_status_filtered && !d_radar[m].fg_fused) {
                data_fused[count].det_mode = DETECT_MODE::RADAR_ONLY;
                data_fused[count].id.x = count;
                data_fused[count].id.z = m;
                data_fused[count].img = NULL;
                data_fused[count].rect_f = cv::Rect();
                data_fused[count].plot_pt_f = d_radar[m].plot_pt_f;
                data_fused[count].pc = d_radar[m].pc_world;
                data_fused[count].pos = SensorBase::polar2Cart(d_radar[m].pc_world);
                data_fused[count].vel = d_radar[m].vel;
                data_fused[count].rect = cv::Rect();
                d_radar[m].fg_fused = true;
                count++;
            }
        }
    }
}

void SensorInfo::drawFusedTopView(bool fg_sv, bool fg_radar, bool fg_sv_each)
{
    stereo_vision::objectInfo *d_sv = sv->objects_display;
    RadarController::objectTrackingInfo *d_radar = rc->objects_display;

    bool fg_update = false;
    if (t.elapsed() > time_gap) {
        fg_update = true;
        lock_f_topview.lockForWrite();
        fused_topview.setTo(cv::Scalar(0, 0, 0, 0));
        lock_f_topview.unlock();
        t.restart();
    }
    else return;

    // Sensor - Stereo vision
    std::string tag;
    int device = SENSOR::SV;
    float scale = 30.0;
    if (fg_sv) {
        lock_f_sv.lockForWrite();
        if (sv->img_r_L.empty())
            sv->img_detected_display = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
        else
            sv->img_detected_display = sv->img_r_L.clone();
        lock_f_sv.unlock();
        for (int k = 0; k < sv->objSize(); k++) {
            if (d_sv[k].labeled) {
                // calculate world range and angle
                int range_precision = 4;
                lock_f_sv.lockForWrite();
                if (d_sv[k].br != std::pair<int, int>(-1, -1) && d_sv[k].tl != std::pair<int, int>(-1, -1)) {
                    cv::rectangle(sv->img_detected_display, cv::Rect(d_sv[k].tl.second, d_sv[k].tl.first, (d_sv[k].br.second - d_sv[k].tl.second), (d_sv[k].br.first - d_sv[k].tl.first)),
                                  d_sv[k].color, sv->thick_obj_rect, 8, 0);
                    cv::circle(sv->img_detected_display, cv::Point(d_sv[k].center.second, d_sv[k].center.first), sv->radius_obj_point, cv::Scalar(0, 255, 0), -1, 8, 0);
                    std::string distance_tag = QString::number(d_sv[k].pc_world.range / 100.0, 'g', range_precision).toStdString() + " M";
                    cv::putText(sv->img_detected_display, distance_tag, cv::Point(d_sv[k].tl.second, d_sv[k].br.first - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 0, 255), 2);
                }
                lock_f_sv.unlock();
            }
        }
    }


    // Fusion
//    if (fg_fusion) {
        lock_f_topview.lockForWrite();
        for (int p = 0 ; p < dataFusedSize(); p++) {
//            int device;
            switch (data_fused[p].det_mode) {
            case DETECT_MODE::SV_RADAR:
                tag = QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness + 2, cv::Scalar(139, 0, 139, 255), -1, 8, 0);
                cv::rectangle(fused_topview, data_fused[p].rect_f, cv::Scalar(255, 255, 0, 255), 1, 8, 0);
                cv::putText(fused_topview, tag, cv::Point(data_fused[p].plot_pt_f.x - 50, data_fused[p].plot_pt_f.y), font, font_size, cv::Scalar(139, 0, 139, 255), font_thickness);
                break;
            case DETECT_MODE::SV_ONLY:
                device = SENSOR::SV;
                tag = QString::number(p).toStdString() + ", " + QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness, sensors[device].color, -1, 8, 0);
                cv::rectangle(fused_topview, data_fused[p].rect_f, cv::Scalar(255, 255, 0, 255), 1, 8, 0);
                cv::putText(fused_topview, tag, data_fused[p].plot_pt_f, font, font_size, sensors[device].color, font_thickness);
                drawArrow(fused_topview, data_fused[p].plot_pt_f, cv::Point(data_fused[p].plot_pt_f.x + scale * ratio * data_fused[p].vel.x, data_fused[p].plot_pt_f.y - scale * ratio * data_fused[p].vel.y), 5, 30, cv::Scalar(255, 255, 0, 255), 1, 8);
                break;
            case DETECT_MODE::RADAR_ONLY:
                device = SENSOR::RADAR;
                tag = QString::number(p).toStdString() + ", " + QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness, sensors[device].color, -1, 8, 0);
                // display range and id
                if (data_fused[p].pc.angle <= sensors[SENSOR::SV].angle_half_fov &&
                        data_fused[p].pc.angle >= -1 * sensors[SENSOR::SV].angle_half_fov && data_fused[p].pc.range < 3000 ||
                        data_fused[p].pc.range < 500) {
                    cv::putText(fused_topview, tag, cv::Point(data_fused[p].plot_pt_f.x, data_fused[p].plot_pt_f.y + 15), font, font_size, sensors[device].color, font_thickness);
                    drawArrow(fused_topview, data_fused[p].plot_pt_f, cv::Point(data_fused[p].plot_pt_f.x + scale * ratio * data_fused[p].vel.x, data_fused[p].plot_pt_f.y - scale * ratio * data_fused[p].vel.y), 5, 30, cv::Scalar(255, 255, 0, 255), 1, 8);
                }
                break;
            }
        }
        lock_f_topview.unlock();
//    }

    // Collision avoidance
    // A*
    if (fg_ca_astar && !ca->path.empty()) {
        lock_f_topview.lockForWrite();
        for (int k = 0; k < ca->path.size(); k++) {
            int map_x = ca->path[k].first;
            int map_y = ca->path[k].second;
            for (int r = map_y; r < map_y + 1; r++) {
                for (int c = map_x; c < map_x + 1; c++) {
                    fused_topview.at<cv::Vec4b>(r, c) = cv::Vec4b(0, 0, 0, 255);
                }
            }
        }
        lock_f_topview.unlock();
    }

    if (fg_update) {
        emit updateGUI(&fused_topview, &sv->img_detected_display);
    }
}

SensorBase::PC SensorInfo::coordinateTransform(int type, cv::Point sensor_pos, SensorBase::PC pc_in)
{
    double range = pc_in.range;
    double angle = pc_in.angle;
    SensorBase::PC pc_out;
    switch (type) {
    case CT_SCS2WCS:
        double x_world, z_world; // (cm)
        x_world = (double)(range * sin(angle * CV_PI / 180.0) + sensor_pos.x);
        z_world = (double)(range * cos(angle * CV_PI / 180.0) + sensor_pos.y - vehicle.head_pos);

        pc_out.angle = atan(x_world / z_world);
        pc_out.range = sqrt(pow(x_world, 2) + pow(z_world, 2));
        break;
    case CT_WCS2SCS:
        break;
    }

    return pc_out;
}

cv::Point SensorInfo::point2FusedTopView(cv::Point sensor_pos, SensorBase::PC pc)
{
    double x_tmp, z_tmp; // (cm)
    x_tmp = pc.range * sin(pc.angle * CV_PI / 180.0);
    z_tmp = pc.range * cos(pc.angle * CV_PI / 180.0);
    int out_x, out_y;
    out_x = vehicle.VCP.x + (x_tmp * ratio + sensor_pos.x);
    out_y = vehicle.VCP.y - (z_tmp * ratio + sensor_pos.y);

    return cv::Point(out_x, out_y);
}

cv::Point SensorInfo::point2FusedTopView(SensorBase::PC pc_world)
{
    double x_tmp, z_tmp; // (cm)
    x_tmp = pc_world.range * sin(pc_world.angle * CV_PI / 180.0);
    z_tmp = pc_world.range * cos(pc_world.angle * CV_PI / 180.0);
    int out_x, out_y;
    out_x = vehicle.VCP.x + (x_tmp * ratio);
    out_y = vehicle.VCP.y - (z_tmp * ratio);

    return cv::Point(out_x, out_y);
}

cv::Rect SensorInfo::rectOnFusedTopView(cv::Point pt_pixel, cv::Rect rect_in)
{
    double w_pixel = rect_in.width * ratio;
    double h_pixel = rect_in.height * ratio;
    return cv::Rect(pt_pixel.x - w_pixel * 0.5, pt_pixel.y - h_pixel * 0.5, w_pixel, h_pixel);
}

int SensorInfo::svDataExec()
{
    int stat = sv->dataExec();

    if (stat == SV::STATUS::OK)
        stat = sv->guiUpdate();

    return stat;
}

int SensorInfo::radarDataExec()
{
    int stat = rc->dataExec();

    if (stat == RADAR::STATUS::OK)
        stat = rc->guiUpdate();

    return stat;
}

bool SensorInfo::lrfDataExec()
{
    bool result = false;
    if (lrf->dataExec()) {
        lrf->guiUpdate();
        result = true;
    }
    return result;
}

void SensorInfo::lrfBufExec()
{
    lrf->pushToBuf();
}

void SensorInfo::drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar& color, int thickness, int lineType)
{
    cv::Point arrow;
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
    cv::line(img, pStart, pEnd, color, thickness, lineType);

    arrow.x = pEnd.x + len * cos(angle + CV_PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle + CV_PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
    arrow.x = pEnd.x + len * cos(angle - CV_PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle - CV_PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
}
