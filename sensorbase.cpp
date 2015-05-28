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
    return SensorBase::PC(sqrt(pow(pt_in.x, 2) + pow(pt_in.y, 2)), atan(pt_in.x / pt_in.y));
}
