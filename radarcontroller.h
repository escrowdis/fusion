#ifndef RADARCONTROLLER_H
#define RADARCONTROLLER_H

#include <QObject>
#include <QTime>
#include <QStandardItemModel>

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock_radar;

// recording
#include "recording/recording.h"
extern recording re;

#include <iostream>
#include <bitset>

// CAN bus lib
#include <canlib.h>

#include <opencv2/opencv.hpp>

// topview
#include "topview.h"

#define OBJECT_NUM 64

namespace RADAR {
enum INPUT_SOURCE {
    ESR,
    TXT
};
}

class RadarController : public QObject, public TopView
{
    Q_OBJECT

public:
    explicit RadarController(float aim_angle);

    ~RadarController();

    bool open();

    bool write();

    void busOn();

    void busOff();

    void retrievingData();

    bool dataExec();

    bool guiUpdate();

    bool fusedTopview() {return fg_topview;}

    int time_proc;

    // status
    int input_mode;                     // process time of all exec.

    // tableView used
    QStandardItem* item;

    float aim_angle;                    // device's aim (yaw) angle (degree)

    bool fg_topview;                    // check wether project to topview

    // smoothing for displaying
    int current_count;

    int update_count;

    struct ESR_track_object_info{
        float angle;                    // azimuth degree. Middle is zero. From -51.2 to 51.1. (degree)
        bool bridge_object;             // non-functional
        bool grouping_changed;
        float lat_rate;
        int med_range_mode;
        bool oncoming;
        float range;                    // (m)
        float range_accel;
        float range_rate;
        bool rolling_count;
        int status;
        float width;                    // non-functional
        double x;                       // radar coordinates system (cm)
        double y;
        double z;
    };

    ESR_track_object_info* esr_obj;

    int detected_obj;

    int obj_status_filtered;            // depends on the status (CAN_TX_TRACK_STATUS)

    // radar frontview image
    cv::Mat img_radar;

    cv::Mat img_radar_BG;

private:
    int* LUT_grid_row;
    int* LUT_grid_col;

    void createLUT();
    int corrGridRow(int k);
    int corrGridCol(int k);
    int col_shift_LUT;

    void reset();

    bool dataIn();

    QTime t_p;                          // process time of all exec.

    const static long id_esr = 0x4F1;
    const static int dlc_esr = 8;

    canHandle h;
    canStatus stat;                 // current CAN status

    QTime t;                        // control gui not to update too fast
    int time_gap;

    bool fg_read;                   // reading data
    bool fg_data_in;                // data is retrievable
    bool fg_all_data_in;

    // CAN bus params ==============
    long            id;
    unsigned char   can_data[8];
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

    // 2D image display ============
    int img_rows;               // frontview image size

    int img_cols;

    int short_length, long_length;

    int short_length_half, long_length_half;

    cv::Point img_center;       // object's center

    cv::Point obj_rect;         // object's rect size

    void drawFrontView();

    void pointDisplayFrontView();
    // ============================= End

    // Topview =====================
    void pointProjectTopView();
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
    void updateGUI(int detected_obj, cv::Mat *img, cv::Mat *topview);

    void dataEnd();
};

#endif // RADARCONTROLLER_H
