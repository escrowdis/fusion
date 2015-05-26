#ifndef OBJECTTRACKING_H
#define OBJECTTRACKING_H

#include "kalmanfilter.h"
#include "particlefilter.h"

class ObjectTracking
{
public:
    ObjectTracking();

    ~ObjectTracking();

    bool isInitialized() {return fg_initialized;}

    int objSize() {return obj_size;}

    void initialze(int object_size);

    // Object List
    struct ObjectTrackingInfo
    {
        cv::Point pos;                  // position at WCS (cm)

        std::pair<float, float> vel;    // velocity of object (cm/s)

        std::pair<float, float> acc;    // acceleration of object (cm/s^2)

        int status;                     // static or dynamic

        KalmanFilter::objectTrackingKF kf;

        ObjectTrackingInfo() {
            vel = std::pair<float, float>(0, 0);

            acc = std::pair<float, float>(0, 0);

            status = -1;

            pos = cv::Point(-1, -1);
        }
    };

    ObjectTrackingInfo *ti;     // object's tracking information

private:
    bool fg_initialized;        // check whether ObjectTrackingInfo is initialized

    int obj_size;               // max size of detected objects
};

#endif // OBJECTTRACKING_H
