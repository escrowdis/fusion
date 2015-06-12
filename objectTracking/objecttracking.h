#ifndef OBJECTTRACKING_H
#define OBJECTTRACKING_H

#include "../debug_info.h"

#include "kalmanfilter.h"
#include "particlefilter.h"

#include "../sensorbase.h"

namespace TRACK_STATUS {
enum {
    NO_TARGET,
    NEW_TARGET,
    UPDATE_TARGET
};
}

namespace DETECT_MODE {
enum {
    NO_DETECT = -1,
    SV_ONLY = 0,
    RADAR_ONLY,
    SV_RADAR
};
}

class ObjectTracking
{
public:
    ObjectTracking();

    ~ObjectTracking();

    void track();

    void reset();

    // object tracking information
    struct objectTrackingInfo
    {
        int det_mode;                   // detected by what kinds of sensors

        cv::Point3i id;                 // each sensors' id for fusion (fused, sv, radar)

        cv::Mat img;                    // object's image

        std::pair<int, int> center;     // Center point of object in image (row, col)

        // FUSED TOPVIEW -----
        cv::Rect rect_f;

        cv::Point plot_pt_f;

        // WCS ===============
        SensorBase::PC pc;              // (cm)

        cv::Point pos;                  // position at WCS (cm)

        cv::Rect rect;                  // (cm)

        cv::Point2f vel;                // velocity of object (cm/s)

        cv::Point2f acc;                // acceleration of object (cm/s^2)

        objectTrackingInfo() {
            det_mode = DETECT_MODE::NO_DETECT;

            id = cv::Point3i(-1, -1, -1);

            pos = cv::Point(-1, -1);

            vel = cv::Point2f(0, 0);

            acc = cv::Point2f(0, 0);
        }
    };

    // Object List
    struct objectTracking
    {
        std::vector<objectTrackingInfo> info;

        std::vector <SensorBase::PC> trajectory;

        cv::Scalar color_trajectory;

        int missed_count;               // detected object was missed for X counts

        bool fg_update;                 // check if the object is updated at this round

        int track_status;               // tracking status

        KalmanFilter::objectTrackingKF kf;

        objectTracking() {
            missed_count = 0;

            fg_update = false;

            track_status = TRACK_STATUS::NO_TARGET;

            info.reserve(200);

            trajectory.reserve(200);
        }
    };

    std::vector<objectTracking> ti;     // object's tracking information

    int thresh_count = 5;               // if object isn't update for this rounds, the data will be clear

    void resetObjectTracking(objectTracking &src);

private:
    bool fg_initialized;        // check whether ObjectTrackingInfo is initialized

    int obj_size;               // max size of detected objects
};

#endif // OBJECTTRACKING_H
