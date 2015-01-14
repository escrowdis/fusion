#include "radarcontroller.h"

RadarController::RadarController() : TopView(1)
{
    fg_read = false;
    fg_data_in = false;

    esr_obj = new ESR_track_object_info[64];
//    qDebug()<<sizeof(esr_obj)<<sizeof(ESR_track_object_info)<<sizeof(int)<<sizeof(float)<<sizeof(bool)<<sizeof(double);

    img_radar = cv::Mat::zeros(250, 300, CV_8UC3);

    topview_BG.create(MAX_DISTANCE, chord_length, CV_8UC4);
    topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    topview.create(MAX_DISTANCE, chord_length, CV_8UC4);
    topview.setTo(cv::Scalar(0, 0, 0, 0));

    time_gap = 10;
    t.start();
}

RadarController::~RadarController()
{
    canClose(h);
}

bool RadarController::open()
{
    canInitializeLibrary();
    h = canOpenChannel(0, canOPEN_NO_INIT_ACCESS);
    stat = canRequestBusStatistics(h);
    if (stat != canOK) {
        char msg[64];
        stat = canGetErrorText(stat, &msg[0], sizeof(msg));
        std::cout<<"ERR"<<std::endl;
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
    if (stat == canOK)
        fg_data_in = true;

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
    stat = canReadWait(h, &id, &data[0], &dlc, &flag, &time, 0xff);
    if (stat != canOK) {
        std::cout<<"read FAILED"<<std::endl;
        return;
    }

//    reset();
    for (int i = 0; i < dlc; i++) {
        b = data[i];
        bin += b.to_string();
    }

    if (fg_data_in) {
        img_radar.setTo(cv::Scalar(0, 0, 0));

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

            if (b_track_lat_rate.at(0) == 1) {
                b_track_lat_rate[0] = 0;
                esr_obj[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25 * -1;
            }
            esr_obj[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25;
            esr_obj[_id].grouping_changed = (b_track_lat_rate.to_ulong() == 1 ? true : false);
            esr_obj[_id].oncoming = (b_track_oncoming.to_ulong() == 1 ? true : false);
            esr_obj[_id].status = b_track_status.to_ulong();
            if (b_track_angle.at(0) == 1) {
                b_track_angle[0] = 0;
                esr_obj[_id].angle = b_track_angle.to_ulong() * 0.1 * -1;
            }
            esr_obj[_id].angle = b_track_angle.to_ulong() * 0.1;
            esr_obj[_id].range = b_track_range.to_ulong() * 0.1;
            esr_obj[_id].bridge_object = (b_track_bridge_object.to_ulong() == 1 ? true : false);
            esr_obj[_id].rolling_count = (b_track_rolling_count.to_ulong() == 1 ? true : false);
            esr_obj[_id].width = b_track_width.to_ulong() * 0.5;
            if (b_track_range_accel.at(0) == 1) {
                b_track_range_accel[0] = 0;
                esr_obj[_id].range_accel = b_track_range_accel.to_ulong() * 0.05 * -1;
            }
            esr_obj[_id].range_accel = b_track_range_accel.to_ulong() * 0.05;
            esr_obj[_id].med_range_mode = b_track_med_range_mode.to_ulong();
            if (b_track_range_rate.at(0) == 1) {
                b_track_range_rate[0] = 0;
                esr_obj[_id].range_rate = b_track_range_rate.to_ulong() * 0.01 * -1;
            }
            esr_obj[_id].range_rate = b_track_range_rate.to_ulong() * 0.01;
        }

//        if (id == 0x53F) {
//            ui->label_3->setText(QString::number(t.elapsed()));
//        }
        int detected_obj = 0;
        for (int k = 0; k < 64; k++) {
            if (esr_obj[k].status != 0) {
                detected_obj++;
                if (esr_obj[k].angle >= 0) {
                    esr_obj[k].x = esr_obj[k].range * cos(abs(esr_obj[k].angle));
                }
                else {
                    esr_obj[k].x = -1.0 * esr_obj[k].range * cos(abs(esr_obj[k].angle));
                }
                esr_obj[k].y = 0.0;
                esr_obj[k].z = esr_obj[k].range * sin(abs(esr_obj[k].angle));
                int shift_y_dis;
                if (esr_obj[k].z > 20)
                    shift_y_dis = esr_obj[k].z * 10 * -1;
                else
                    shift_y_dis = esr_obj[k].z * 10;

                cv::circle(img_radar, cv::Point(esr_obj[k].x + 150, 100 + shift_y_dis), 1, cv::Scalar(0, 255, 0), -1, 8, 0);
                cv::rectangle(img_radar, cv::Rect(esr_obj[k].x - 20 + 150, 80 + shift_y_dis, 40, 40), cv::Scalar(0, 0, 255), 2, 8, 0);
                cv::putText(img_radar, QString::number(k + 1).toStdString(), cv::Point(esr_obj[k].x + 150, 160 + shift_y_dis), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));

//                ui->textEdit->append("data struct\nangle: " + QString::number(esr_obj[k].angle) + " range: " + QString::number(esr_obj[k].range)
//                                     + " accel: " + QString::number(esr_obj[k].range_accel) + " width: " + QString::number(esr_obj[k].width)
//                                     + " (x,y,z) = (" + QString::number(esr_obj[k].x) + ", " + QString::number(esr_obj[k].y) + ", " + QString::number(esr_obj[k].z) + ")");
//                item[k].setText(QString::number(esr_obj[k].range));
            }
//            else {
//                item[k].setText("0");
//            }
        }

        if (t.elapsed() > 10) {
            emit radarUpdateGUI(&img_radar, detected_obj);
//            pointProjectTopView(esr_obj, color_table);
            t.restart();
        }
    }
}

void RadarController::pointProjectTopView(ESR_track_object_info *data, QImage *color_table)
{
    resetTopView();

    double va = 70.7;
    int grid_row, grid_col;
    for (int m = 0; m < 64; m++) {
        if (esr_obj[m].status != 0) {
            //esr_obj[k].x + 150
            grid_row = 1.0 * log10(100.0 * data[m].z / MIN_DISTANCE) / log10(1.0 + k);
            grid_col = 360.0 * img_col_half * atan((c / (double)(320.0 / img_col) - img_col_half) / (data[m].z * 100.0)) / (va * CV_PI) + img_col_half;
//            std::cout<<data[m].z<<" "<<data[m].x<<"\t"<<grid_row<<" "<<grid_col<<std::endl;
            // mark each point belongs to which cell
            if (grid_row >= 0 && grid_row < img_row &&
                    grid_col >= 0 && grid_col < img_col) {
                grid_map[img_row - grid_row - 1][grid_col]++;
            }
        }
    }
    // check whether the cell is satisfied as an object
    cv::Point pts[4];
    uchar* ptr = color_table->scanLine(0);
    int p;
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            if (grid_map[r][c] >= thresh_free_space) {
                int ch_row = img_row - (r + 5) > img_row ? img_row : img_row - (r + 5);
                int ch_col = c + 5 > img_col ? img_col : c + 5;
                pts[0] = img_grid[img_row - r][c];
                pts[1] = img_grid[ch_row][c];
                pts[2] = img_grid[ch_row][ch_col];
                pts[3] = img_grid[img_row - r][ch_col];
                p = (MAX_DISTANCE - 0.5 * (pts[0].y + pts[1].y)) - MIN_DISTANCE;

                cv::fillConvexPoly(topview, pts, 4, cv::Scalar(ptr[3 * p + 0], ptr[3 * p + 1], ptr[3 * p + 2], 255), 8, 0);
                cv::circle(topview, pts[0], 5, cv::Scalar(ptr[3 * p + 0], ptr[3 * p + 1], ptr[3 * p + 2], 255), -1, 8, 0);
            }
        }
    }
}
