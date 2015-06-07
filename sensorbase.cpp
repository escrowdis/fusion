#include "sensorbase.h"

SensorBase::SensorBase()
{
}

cv::Point SensorBase::polar2Cart(SensorBase::PC pc_in)
{
    return cv::Point((int)(pc_in.range * sin(pc_in.angle * CV_PI / 180.0)), (int)((pc_in.range * cos(pc_in.angle * CV_PI / 180.0))));
}

SensorBase::PC SensorBase::cart2Polar(cv::Point pt_in)
{
    float angle;
    if (pt_in.y == 0)
        angle = CV_PI / 2;
    else
        angle = atan(pt_in.x / pt_in.y);

    return SensorBase::PC(sqrt(pow(pt_in.x, 2) + pow(pt_in.y, 2)), angle);
}

cv::Point2f SensorBase::velEstimation(cv::Point2f p_now, cv::Point2f p_prev, int time_proc)
{
    float frame2sec = 1000.0 / (1.0 * time_proc);
    float m2km_s2hr = 3600.0 / 1000.0;
    return cv::Point2f(1.0 * (p_now.x - p_prev.x) * m2km_s2hr * frame2sec,
                       1.0 * (p_now.y - p_prev.y) * m2km_s2hr * frame2sec);
}
