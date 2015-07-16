#ifndef SENSORBASE_H
#define SENSORBASE_H

#include "opencv2/core.hpp"

struct VEL_ESTI {
    enum {
        M_PER_FRAME2M_PER_SEC,
        M_PER_FRAME2KM_PER_HR,
        CM_PER_FRAME2CM_PER_SEC,
        CM_PER_FRAME2M_PER_SEC,
        CM_PER_FRAME2KM_PER_HR
    };
};

struct VEL_UNIT {
    enum {
        M_PER_SEC2KM_PER_HR,
        CM_PER_SEC2KM_PER_HR,
        KM_PER_HR2M_PRE_SEC,
        KM_PER_HR2CM_PER_SEC
    };
};

class SensorBase
{
public:
    SensorBase();

    // Polar Coordinates System
    struct PC
    {
        double angle;                   // orientation degree (degree).
                                        // Forward is set as zero and counting clockwisely.

        double range;                   // (cm)

        PC(double range = 0.0, double angle = 0.0) {
            this->angle = angle;
            this->range = range;
        }
    };

    // Conversion between polar and Cartesian
    static cv::Point2d polar2Cartf(PC pc_in);
    static cv::Point polar2Cart(PC pc_in);
    static SensorBase::PC cart2Polarf(cv::Point2f pt_in);
    static PC cart2Polar(cv::Point pt_in);
    static cv::Point2f velUnitTransform(cv::Point2f src, int type);
    static cv::Point2f velEstimation(cv::Point2f p_now, cv::Point2f p_prev, float time_proc, int type);

};

#endif // SENSORBASE_H
