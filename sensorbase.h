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
        double angle;                    // orientation degree. Middle is zero. (degree)
        double range;                    // (cm)
        PC(double range = 0.0, double angle = 0.0) {
            this->angle = angle;
            this->range = range;
        }
    };

    struct sensorLocation {
        PC pc;

        cv::Point pos;                              // sensor's position based on the center of vehicle
                                                    // (cm) forward is x+, lateral right is y+.

        float theta;                                // the orientation of sensor (degree).                                                    // foward is 0 and count clockwise
    };
};

#endif // SENSORBASE_H
