#ifndef RADARCONTROLLER_H
#define RADARCONTROLLER_H

#include <QObject>
#include <QTime>
#include <QThread>

#include <iostream>
#include <bitset>

// CAN bus lib
#include <canlib.h>

#include <opencv2/opencv.hpp>

// topview
#include "topview.h"

class RadarController : public QObject, public TopView
{
    Q_OBJECT

public:
    explicit RadarController();

    ~RadarController();

    bool open();

    bool write();

    void busOn();

    void busOff();

    void retrievingData();

    int count_obj = 0;

    struct ESR_track_object_info{
        float angle;
        bool bridge_object;
        bool grouping_changed;
        float lat_rate;
        int med_range_mode;
        bool oncoming;
        float range;
        float range_accel;
        float range_rate;
        bool rolling_count;
        int status;
        float width;
        double x;
        double y;
        double z;
    };

    ESR_track_object_info* esr_obj;

    cv::Mat img_radar;

    // Topview =====================
    void pointProjectTopView(ESR_track_object_info *data, QImage *color_table);
    // ============================= End

private:
    void reset();

    const static long id_esr = 0x4F1;
    const static int dlc_esr = 8;

    canHandle h;
    canStatus stat;                 // current CAN status

    QTime t;                        // control gui not to update too fast
    int time_gap;

    bool fg_read;                   // reading data
    bool fg_data_in;                // data is retrievable

    // CAN bus params ==============
    long            id;
    unsigned char   data[8];
    unsigned int    dlc;
    unsigned int    flag;
    DWORD           time;

    std::bitset<8> b;
    std::string bin;
    std::bitset<6> b_track_lat_rate;
    std::bitset<1> b_track_grouping_changed;
    std::bitset<1> b_track_oncoming;
    std::bitset<3> b_track_status;
    std::bitset<10> b_track_angle;
    std::bitset<11> b_track_range;
    std::bitset<1> b_track_bridge_object;
    std::bitset<1> b_track_rolling_count;
    std::bitset<4> b_track_width;
    std::bitset<10> b_track_range_accel;
    std::bitset<2> b_track_med_range_mode;
    std::bitset<14> b_track_range_rate;
    // ============================= End

    struct ESR_STAT {
        struct BRIDGE_OBJECT {
            enum {
                NOT_A_BRIDGE = false,
                BRIDGE = true
            };
        };

        struct GROUPING_CHANGED {
            enum {
                NO = false,
                YES = true
            };
        };

        struct MED_RANGE_MODE {
            enum {
                BOTH_NO_UPDATE,
                MR_UPDATE,
                LR_UPDATE,
                BOTH_UPDATE
            };
        };

        struct ONCOMING {
            enum {
                NO = false,
                YES = true
            };
        };

        //**// rolling_count

        struct STATUS {
            enum {
                NO_TARGET,
                NEW_TARGET,
                NEW_UPDATED_TARGET,
                UPDATED_TARGET,
                COASTED_TARGET,
                MERGED_TARGET,
                INVALID_COASTED_TARGET,
                NEW_COASTED_TARGET
            };
        };
    };

signals:
    void radarUpdateGUI(cv::Mat *img, int detected_obj);
};

#endif // RADARCONTROLLER_H
