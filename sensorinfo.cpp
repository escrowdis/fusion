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

    pic_sv = QPixmap(20, 20);
    pic_radar = QPixmap(20, 20);
}

SensorInfo::~SensorInfo()
{
    delete sv;
    delete rc;
    delete lrf;
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

    vehicle.rect = cv::Rect(vehicle.VCP.x - (int)(vehicle.width * ratio / 2), vehicle.VCP.y - (int)(vehicle.length * ratio / 2),
                            (int)(vehicle.width * ratio), (int)(vehicle.length * ratio));

    vehicle.color = cv::Scalar(0, 255, 0, 255);

    fused_topview.setTo(cv::Scalar(0, 0, 0, 0));

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

void SensorInfo::drawFusedTopView(bool fg_sv, bool fg_sv_each, bool fg_radar)
{
    stereo_vision::objInformation *d_sv = sv->objects_display;
    RadarController::ESR_track_object_info *d_radar = rc->esr_obj;
    fused_topview.setTo(cv::Scalar(0, 0, 0, 0));

    std::string tag;
    int device = SENSOR::SV;
    int range_precision = 3;
    if (fg_sv) {
        // every pixel
        if (fg_sv_each) {
            cv::Scalar color_pixel = sensors[device].color;
            color_pixel[3] = 100;
            for (int r = 0; r < IMG_H; r++) {
                for (int c = 0; c < IMG_W; c++) {
                    if (sv->data[r][c].disp > 0) {
                        cv::Point plot_pt;
                        float range, angle;
                        range = sqrt(pow((double)(sv->data[r][c].Z), 2) + pow((double)(sv->data[r][c].X), 2));
                        angle = atan(1.0 * sv->data[r][c].X / (1.0 * sv->data[r][c].Z)) * 180.0 / CV_PI;
                        pointTransformTopView(sensors[device].pos_pixel, range, angle, &plot_pt);
                        cv::circle(fused_topview, plot_pt, 1, color_pixel, -1, 8, 0);
                    }
                }
            }
        }
        // objects
        lock_f_sv.lockForWrite();
        if (sv->img_r_L.empty())
            sv->img_detected_display = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
        else
            sv->img_detected_display = sv->img_r_L.clone();
        lock_f_sv.unlock();
        for (int k = 0; k < sv->objSize(); k++) {
            if (d_sv[k].labeled) {
                cv::Point plot_pt;
                cv::Rect rect;
                PC pc = PC(d_sv[k].range, d_sv[k].angle);
                float range_world = pointTransformTopView(sensors[device].pos_pixel, pc.range, pc.angle, &plot_pt, d_sv[k].rect, &rect);
                cv::circle(fused_topview, plot_pt, thickness, sensors[device].color, -1, 8, 0);
                cv::rectangle(fused_topview, rect, cv::Scalar(255, 255, 0, 255), 1, 8, 0);
                tag = QString::number(k).toStdString() + ", " + QString::number(range_world / 100, 'g', range_precision).toStdString();
                cv::putText(fused_topview, tag, plot_pt, font, font_size, sensors[device].color, font_thickness);

                // calculate world range and angle
                int range_precision = 3;
                if (d_sv[k].br != std::pair<int, int>(-1, -1) && d_sv[k].tl != std::pair<int, int>(-1, -1)) {
                    PC pc_world = rangeWorldCalculation(sensors[SENSOR::SV].location.pos, pc);

                    lock_f_sv.lockForWrite();
                    cv::rectangle(sv->img_detected_display, cv::Rect(d_sv[k].tl.second, d_sv[k].tl.first, (d_sv[k].br.second - d_sv[k].tl.second), (d_sv[k].br.first - d_sv[k].tl.first)),
                                  d_sv[k].color, sv->thick_obj_rect, 8, 0);
                    cv::circle(sv->img_detected_display, cv::Point(d_sv[k].center.second, d_sv[k].center.first), sv->radius_obj_point, cv::Scalar(0, 255, 0), -1, 8, 0);
                    std::string distance_tag = QString::number(pc_world.range / 100.0, 'g', range_precision).toStdString() + " M";
                    cv::putText(sv->img_detected_display, distance_tag, cv::Point(d_sv[k].tl.second, d_sv[k].br.first - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 0, 255), 2);
                    lock_f_sv.unlock();
                }


                // Fusion ================================
                double U_D = 500;    // max distance error (cm)
                double R_sv = U_D * range_world / sv->max_distance;  // (cm)
                int closest_radar_id = -1;
                double closest_radar_distance = 10000000.0;
                double sv_x = pc.range * sin(pc.angle * CV_PI / 180.0) + sensors[0].location.pos.x;
                double sv_y = pc.range * cos(pc.angle * CV_PI / 180.0) + sensors[0].location.pos.y - vehicle.head_pos;
                double radar_min_x;
                double radar_min_y;
//                std::cout<<"SV: "<<sv_x<<" "<<sv_y<<std::endl;
                if (fg_radar) {
                    for (int m = 0 ; m < rc->objSize(); m++) {
                        if (d_radar[m].status >= rc->obj_status_filtered) {
                            double radar_x = 100.0 * d_radar[m].range * sin(d_radar[m].angle * CV_PI / 180.0) + sensors[1].location.pos.x;
                            double radar_y = 100.0 * d_radar[m].range * cos(d_radar[m].angle * CV_PI / 180.0) + sensors[1].location.pos.y - vehicle.head_pos;
                            double deviation = sqrt(pow((double)(radar_x - sv_x), 2) + pow((double)(radar_y - sv_y), 2));
//                            std::cout<<deviation<<" "<<closest_radar_distance<<" "<<R_sv<<std::endl;
//                            std::cout<<"RADAR: "<<radar_x<<" "<<radar_y<<std::endl;
                            if (deviation < closest_radar_distance && deviation < R_sv) {
                                closest_radar_id = m;
                                closest_radar_distance = deviation;
                                radar_min_x = radar_x;
                                radar_min_y = radar_y;
                            }
//                            std::cout<<"dist. "<<sv_x<<" "<<sv_y<<"\t"<<radar_x<<" "<<radar_y<<std::endl;
                        }
                    }

                    if (closest_radar_id != -1) {
                        cv::Point2d fused_pos;
                        float ratio_radar = 0.9;
                        float ratio_sv = 0.1;
                        double fused_x = vehicle.VCP.x + (ratio_radar * radar_min_x + ratio_sv * sv_x) * ratio;
                        double fused_y = vehicle.VCP.y - (ratio_radar * radar_min_y + ratio_sv * sv_y) * ratio;
                        double range = sqrt(pow((double)(0.1 * (sv_x + sensors[0].location.pos.x) + 0.9 * (radar_min_x + sensors[0].location.pos.x)), 2) +
                                            pow((double)(0.1 * (sv_y + sensors[1].location.pos.y) + 0.9 * (radar_min_y + sensors[1].location.pos.y) - vehicle.head_pos), 2));
                        fused_pos = cv::Point2d(fused_x, fused_y);
                        cv::circle(fused_topview, cv::Point(fused_pos), thickness + 2, cv::Scalar(139, 0, 139, 255), -1, 8, 0);
                        tag = QString::number(range / 100, 'g', range_precision).toStdString();
                        cv::putText(fused_topview, tag, cv::Point(plot_pt.x - 50, plot_pt.y), font, font_size, cv::Scalar(139, 0, 139, 255), font_thickness);
//                        std::cout<<k<<" "<<closest_radar_id<<"\t"<<range<<std::endl;
                    }
                }
            }
        }
    }

    device = SENSOR::RADAR;
    if (fg_radar) {
        for (int m = 0; m < rc->objSize(); m++) {
            if (d_radar[m].status >= rc->obj_status_filtered) {
                cv::Point plot_pt;
                float range_world = pointTransformTopView(sensors[device].pos_pixel, 100 * d_radar[m].range, d_radar[m].angle + rc->aim_angle, &plot_pt);
                cv::circle(fused_topview, plot_pt, thickness, sensors[device].color, -1, 8, 0);
                tag = QString::number(m).toStdString() + ", " + QString::number(range_world / 100, 'g', range_precision).toStdString();
                cv::putText(fused_topview, tag, cv::Point(plot_pt.x, plot_pt.y + 15), font, font_size, sensors[device].color, font_thickness);
            }
        }
    }


    emit updateGUI(&fused_topview, &sv->img_detected_display);
}

double SensorInfo::pointTransformTopView(cv::Point sensor_pos, double range, double angle, cv::Point *output)
{
    double x_tmp, y_tmp; // (cm)
    x_tmp = range * sin(angle * CV_PI / 180.0);
    y_tmp = range * cos(angle * CV_PI / 180.0);
    output->x = vehicle.VCP.x + (x_tmp * ratio + sensor_pos.x);
    output->y = vehicle.VCP.y - (y_tmp * ratio + sensor_pos.y);

    return sqrt(pow((double)(x_tmp + sensor_pos.x / ratio), 2) + pow((double)(y_tmp + sensor_pos.y / ratio - vehicle.head_pos), 2));
}

double SensorInfo::pointTransformTopView(cv::Point sensor_pos, double range, double angle, cv::Point *output, cv::Rect rect_in, cv::Rect *rect)
{
    double x_tmp, z_tmp; // (cm)
    x_tmp = range * sin(angle * CV_PI / 180.0);
    z_tmp = range * cos(angle * CV_PI / 180.0);
    output->x = vehicle.VCP.x + (x_tmp * ratio + sensor_pos.x);
    output->y = vehicle.VCP.y - (z_tmp * ratio + sensor_pos.y);
    *rect = cv::Rect(vehicle.VCP.x + (rect_in.tl().x * ratio + sensor_pos.x),
                     vehicle.VCP.y - (rect_in.tl().y * ratio + sensor_pos.y),
                     (rect_in.br().x - rect_in.tl().x) * ratio, abs(rect_in.br().y - rect_in.tl().y) * ratio);

//    std::cout<<rect->tl().x<<"\t"<<rect->tl().y<<"\t"<<rect->width<<"\t"<<rect->height<<"\t\t"<<
//            rect_in.tl().x<<"\t"<<rect_in.tl().y<<"\t"<<rect_in.width<<"\t"<<rect_in.height<<std::endl;

    return sqrt(pow((double)(x_tmp + sensor_pos.x / ratio), 2) + pow((double)(z_tmp + sensor_pos.y / ratio - vehicle.head_pos), 2));
}

SensorBase::PC SensorInfo::rangeWorldCalculation(cv::Point sensor_pos, SensorBase::PC pc)
{
    double range = pc.range;
    double angle = pc.angle;
    double x_world, z_world; // (cm)
    x_world = (double)(range * sin(angle * CV_PI / 180.0) + sensor_pos.x);
    z_world = (double)(range * cos(angle * CV_PI / 180.0) + sensor_pos.y - vehicle.head_pos);

    SensorBase::PC pc_tmp;
    pc_tmp.angle = atan(x_world / z_world);
    pc_tmp.range = sqrt(pow(x_world, 2) + pow(z_world, 2));
    return pc_tmp;
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
