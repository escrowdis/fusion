#include "radarcontroller.h"

RadarController::RadarController(float aim_angle) : TopView(1, 100, 20470, 102.3, 31900, 600, 900, 300, 400)//TopView(1, 200, 3000, 19.8, 1080, 750, 270, 125, 100)
{
    this->aim_angle = aim_angle;

    fg_read = false;
    fg_data_in = false;

    esr_obj = new ESR_track_object_info[64];

    obj_status_filtered = 2;

    item = new QStandardItem[64];

    img_rows = 160;

    img_cols = 900;

    img_center = cv::Point(img_cols / 2, img_rows / 2);

    obj_rect = cv::Point(20, 40);

    img_radar = cv::Mat::zeros(img_rows, img_cols, CV_8UC4);

    img_radar_BG = cv::Mat::zeros(img_rows, img_cols, CV_8UC3);

    drawFrontView();

    current_count = 0;

    update_count = 3;

    time_gap = 30;
    t.start();
}

RadarController::~RadarController()
{
    canClose(h);

    delete[] esr_obj;

    delete[] item;
}

bool RadarController::open()
{
    canInitializeLibrary();
    h = canOpenChannel(0, canOPEN_NO_INIT_ACCESS);
    stat = canRequestBusStatistics(h);
    if (stat != canOK) {
//        char msg[64];
//        stat = canGetErrorText(stat, &msg[0], sizeof(msg));
//        std::cout<<"ERR"<<std::endl;
        return false;
    }

    return true;
}

bool RadarController::write()
{
    BYTE msg[dlc_esr] = {0x00};
    msg[6] = 0xBF;
    stat = canWriteWait(h, id_esr, msg, dlc_esr, 0, 0xff);
    std::cout<<"write stat: "<<stat<<std::endl;
    if (stat == canOK) {
        fg_data_in = true;
        return true;
    }

    return false;
}

void RadarController::reset()
{
    bin.clear();
    b.reset();
    b_track_lat_rate.reset();
    b_track_grouping_changed.reset();
    b_track_oncoming.reset();
    b_track_status.reset();
    b_track_angle.reset();
    b_track_range.reset();
    b_track_bridge_object.reset();
    b_track_rolling_count.reset();
    b_track_width.reset();
    b_track_range_accel.reset();
    b_track_med_range_mode.reset();
    b_track_range_rate.reset();
}

void RadarController::busOn()
{
    stat = canBusOn(h);

    if (stat == canOK)
        fg_read = true;
}

void RadarController::busOff()
{
    fg_read = false;
    stat = canBusOff(h);
}

void RadarController::retrievingData()
{
    stat = canReadWait(h, &id, &can_data[0], &dlc, &flag, &time, 0xff);
    if (stat != canOK) {
        std::cout<<"read FAILED"<<std::endl;
        return;
    }

    reset();
    for (int i = 0; i < dlc; i++) {
        b = can_data[i];
        bin += b.to_string();
    }

    if (fg_data_in) {
        int _id;
        if (id >= 0x500 && id <= 0x53F) {
            _id = id - 0x500;

            b_track_lat_rate = std::bitset<6>(bin.substr(0, 6));
            b_track_oncoming = std::bitset<1>(bin.substr(6, 1));
            b_track_grouping_changed = std::bitset<1>(bin.substr(7, 1));
            b_track_status = std::bitset<3>(bin.substr(8, 3));
            b_track_angle = std::bitset<10>(bin.substr(11, 10));
            b_track_range = std::bitset<11>(bin.substr(21, 11));
            b_track_bridge_object = std::bitset<1>(bin.substr(32, 1));
            b_track_rolling_count = std::bitset<1>(bin.substr(33, 1));
            b_track_width = std::bitset<4>(bin.substr(34, 4));
            b_track_range_accel = std::bitset<10>(bin.substr(38, 10));
            b_track_med_range_mode = std::bitset<2>(bin.substr(48, 2));
            b_track_range_rate = std::bitset<14>(bin.substr(50, 14));

#ifdef debug_info_radar_data
            std::cout<<_id<<"\t\n"<<
                    b_track_lat_rate <<" "<<
                    b_track_oncoming <<" "<<
                    b_track_grouping_changed <<" "<<
                    b_track_status <<" "<<
                    b_track_angle <<" "<<
                    b_track_range <<" "<<
                    b_track_bridge_object <<" "<<
                    b_track_rolling_count <<" "<<
                    b_track_width <<" "<<
                    b_track_range_accel <<" "<<
                    b_track_med_range_mode <<" "<<
                    b_track_range_rate <<" "<<
                    std::endl;
#endif

            if (b_track_lat_rate.at(5) == 1) {
                b_track_lat_rate[5] = 0;
                esr_obj[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25 - 8.0;
            }
            else
                esr_obj[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25;
            esr_obj[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25;
            esr_obj[_id].grouping_changed = (b_track_lat_rate.to_ulong() == 1 ? true : false);
            esr_obj[_id].oncoming = (b_track_oncoming.to_ulong() == 1 ? true : false);
            esr_obj[_id].status = b_track_status.to_ulong();
            if (b_track_angle.at(9) == 1) {
                b_track_angle[9] = 0;
                esr_obj[_id].angle = b_track_angle.to_ulong() * 0.1 - 51.2;
            }
            else
                esr_obj[_id].angle = b_track_angle.to_ulong() * 0.1;
            esr_obj[_id].range = b_track_range.to_ulong() * 0.1;
            esr_obj[_id].bridge_object = (b_track_bridge_object.to_ulong() == 1 ? true : false);
            esr_obj[_id].rolling_count = (b_track_rolling_count.to_ulong() == 1 ? true : false);
            esr_obj[_id].width = b_track_width.to_ulong() * 0.5;
            if (b_track_range_accel.at(9) == 1) {
                b_track_range_accel[9] = 0;
                esr_obj[_id].range_accel = b_track_range_accel.to_ulong() * 0.05 - 25.6;
            }
            else
                esr_obj[_id].range_accel = b_track_range_accel.to_ulong() * 0.05;
            esr_obj[_id].med_range_mode = b_track_med_range_mode.to_ulong();
            if (b_track_range_rate.at(13) == 1) {
                b_track_range_rate[13] = 0;
                esr_obj[_id].range_rate = b_track_range_rate.to_ulong() * 0.01 - 81.92;
            }
            else
                esr_obj[_id].range_rate = b_track_range_rate.to_ulong() * 0.01;

            esr_obj[_id].x = 1.0 * esr_obj[_id].range * sin(abs(esr_obj[_id].angle));
            esr_obj[_id].y = 0.0;
            esr_obj[_id].z = esr_obj[_id].range * cos(esr_obj[_id].angle);

#ifdef debug_info_radar_data
            std::cout<<_id<<"\t\n"<<
                    esr_obj[_id].lat_rate <<" "<<
                    esr_obj[_id].oncoming <<" "<<
                    esr_obj[_id].grouping_changed <<" "<<
                    esr_obj[_id].status <<" "<<
                    esr_obj[_id].angle <<" "<<
                    esr_obj[_id].range <<" "<<
                    esr_obj[_id].bridge_object <<" "<<
                    esr_obj[_id].rolling_count <<" "<<
                    esr_obj[_id].width <<" "<<
                    esr_obj[_id].range_accel <<" "<<
                    esr_obj[_id].med_range_mode <<" "<<
                    esr_obj[_id].range_rate <<" "<<
                    std::endl<<std::endl;
#endif
        }
#ifdef debug_info_radar_data
        if (id == 0x53F) {
            qDebug()<<t.elapsed();
        }
#endif

        pointDisplayFrontView();

        if (fg_topview)
            pointProjectTopView();

        if (t.elapsed() > time_gap) {
            emit radarUpdateGUI(detected_obj, &img_radar, &topview);

            t.restart();
        }
    }
}

void RadarController::drawFrontView()
{
    short_length = (img_cols / 4);
    long_length = img_cols;
    short_length_half = short_length / 2;
    long_length_half = long_length / 2;
    cv::line(img_radar_BG, cv::Point(img_center.x + short_length_half, 0), cv::Point(img_center.x + long_length_half, img_rows), cv::Scalar(255, 255, 255), 1, 8, 0);
    cv::line(img_radar_BG, cv::Point(img_center.x - short_length_half, 0), cv::Point(img_center.x - long_length_half, img_rows), cv::Scalar(255, 255, 255), 1, 8, 0);
}

void RadarController::pointDisplayFrontView()
{
    img_radar.setTo(cv::Scalar(0, 0, 0, 0));

    detected_obj = 0;

    for (int k = 0; k < 64; k++) {
        if (esr_obj[k].status >= obj_status_filtered) {
            detected_obj++;

            int pt_x, pt_y;
            float gain = 5.0; //**// another value?
            lock.lockForRead();
            pt_x = 1.0 * esr_obj[k].x * gain * (1.0 - 1.0 * esr_obj[k].z / max_distance) + img_center.x;
            pt_y = img_rows * (1.0 - 1.0 * esr_obj[k].z / max_distance);
            cv::circle(img_radar, cv::Point(pt_x, pt_y), 1, cv::Scalar(0, 255, 0, 255), -1, 8, 0);
            cv::rectangle(img_radar, cv::Rect(pt_x - obj_rect.x / 2, pt_y - obj_rect.y / 2, obj_rect.x, obj_rect.y), cv::Scalar(0, 0, 255, 255), 2, 8, 0);
            cv::putText(img_radar, QString::number(k + 1).toStdString(), cv::Point(pt_x + obj_rect.x / 2, pt_y), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0, 255));
            lock.unlock();
#ifdef debug_info_radar_data
//                ui->textEdit->append("data struct\nangle: " + QString::number(esr_obj[k].angle) + " range: " + QString::number(esr_obj[k].range)
//                                     + " accel: " + QString::number(esr_obj[k].range_accel) + " width: " + QString::number(esr_obj[k].width)
//                                     + " (x,y,z) = (" + QString::number(esr_obj[k].x) + ", " + QString::number(esr_obj[k].y) + ", " + QString::number(esr_obj[k].z) + ")");
#endif
            item[k].setText(QString::number(esr_obj[k].range));
        }
        else {
            item[k].setText("0");
        }
    }
}

void RadarController::pointProjectTopView()
{
    if (current_count > update_count) {
        resetTopView();
        current_count = 0;
    }

    int grid_row, grid_col;
    for (int m = 0; m < 64; m++) {
        if (esr_obj[m].status >= obj_status_filtered) {
            grid_row = 1.0 * log10(100.0 * esr_obj[m].z / min_distance) / log10(1.0 + k);
            grid_col = (100.0 * esr_obj[m].x + 0.5 * chord_length) * img_col / chord_length;
            int grid_row_t = img_row - grid_row - 1;
            int grid_col_t = grid_col;
            // mark each point belongs to which cell
            if (grid_row_t >= 0 && grid_row_t < img_row &&
                    grid_col_t >= 0 && grid_col_t < img_col) {
                lock.lockForWrite();
                grid_map[grid_row_t][grid_col_t].pts_num++;
//                std::cout<<data[m].z<<std::endl;
                lock.unlock();
            }
        }
    }
    // check whether the cell is satisfied as an object
    cv::Point pts[4];
    uchar* ptr = color_table->scanLine(0);
    int p;
    int gap_row  = 3;
    int gap_col  = 5;
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            if (grid_map[r][c].pts_num >= thresh_free_space) {
                int row = img_row - r - gap_row > 0 ? img_row - r - gap_row : 0;
                int row_1 = img_row - (r + 1) + gap_row <= img_row ? img_row - (r + 1) + gap_row : img_row;
                int col = c - gap_col > 0 ? c - gap_col : 0;
                int col_1 = c + 1 + gap_col <= img_col ? c + 1 + gap_col : img_col;

                pts[0] = pointT(img_grid[row][col]);
                pts[1] = pointT(img_grid[row_1][col]);
                pts[2] = pointT(img_grid[row_1][col_1]);
                pts[3] = pointT(img_grid[row][col_1]);

                p = (max_distance - 0.5 * (img_grid[row][col].y + img_grid[row_1][col].y)) - min_distance;

//                cv::fillConvexPoly(topview, pts, thick_polygon, cv::Scalar(ptr[3 * p + 0], ptr[3 * p + 1], ptr[3 * p + 2], 255), 8, 0);
                cv::fillConvexPoly(topview, pts, thick_polygon, cv::Scalar(255, 255, 255, 255), 8, 0);
            }
        }
    }

    current_count++;
#ifdef debug_info_radar_data
    if (current_count >= 10) {
        cv::Mat tp;
        tp = topview.clone();
        //    cv::transpose(topview, tp);
        cv::imshow("topview - rc", tp);
    }
#endif
}
