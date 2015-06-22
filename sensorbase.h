#ifndef SENSORBASE_H
#define SENSORBASE_H

#include "opencv2/core.hpp"

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
    static PC cart2Polar(cv::Point pt_in);
    static cv::Point2f velEstimation(cv::Point2f p_now, cv::Point2f p_prev, double time_proc);

};

#endif // SENSORBASE_H
