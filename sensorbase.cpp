#include "sensorbase.h"

SensorBase::SensorBase()
{
}

cv::Point2d SensorBase::polar2Cartf(SensorBase::PC pc_in)
{
    return cv::Point2d(pc_in.range * sin(pc_in.angle * CV_PI / 180.0), (pc_in.range * cos(pc_in.angle * CV_PI / 180.0)));
}

cv::Point SensorBase::polar2Cart(SensorBase::PC pc_in)
{
    return cv::Point((int)(pc_in.range * sin(pc_in.angle * CV_PI / 180.0)), (int)((pc_in.range * cos(pc_in.angle * CV_PI / 180.0))));
}

SensorBase::PC SensorBase::cart2Polarf(cv::Point2f pt_in)
{
    float angle;
    if (pt_in.y == 0)
        angle = CV_PI / 2;
    else
        angle = atan(pt_in.x / pt_in.y);

    return SensorBase::PC(sqrt(pow(pt_in.x, 2) + pow(pt_in.y, 2)), angle);
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

cv::Point2f SensorBase::velEstimation(cv::Point2f p_now, cv::Point2f p_prev, float time_proc, int type)
{
    float frame2sec = 1000.0 / (1.0 * time_proc);
    float cm2m = 0.01;
    float cm2km = 0.00001;
    float m2km = 0.001;
    float sec2hr = 3600;        // 1/sec -> 1/hr
    cv::Point2f output;
    switch (type) {
    // (cm/frame) -> (km/hr)
    case VEL_ESTI::CM_PER_FRAME2KM_PER_HR:
        output = cv::Point2f(1.0 * (p_now.x - p_prev.x) * frame2sec * sec2hr * cm2km,
                             1.0 * (p_now.y - p_prev.y) * frame2sec * sec2hr * cm2km);
        break;
    // (cm/frame) -> (m/s)
    case VEL_ESTI::CM_PER_FRAME2M_PER_SEC:
        output = cv::Point2f(1.0 * (p_now.x - p_prev.x) * frame2sec * cm2m,
                             1.0 * (p_now.y - p_prev.y) * frame2sec * cm2m);
        break;
    // (m/frame) -> (km/hr)
    case VEL_ESTI::M_PER_FRAME2KM_PER_HR:
        output = cv::Point2f(1.0 * (p_now.x - p_prev.x) * frame2sec * sec2hr * m2km,
                             1.0 * (p_now.y - p_prev.y) * frame2sec * sec2hr * m2km);
        break;
    // (m/frame) -> (m/s) & (cm/frame) -> (cm/s)
    case VEL_ESTI::M_PER_FRAME2M_PER_SEC:
    case VEL_ESTI::CM_PER_FRAME2CM_PER_SEC:
        output = cv::Point2f(1.0 * (p_now.x - p_prev.x) * frame2sec,
                             1.0 * (p_now.y - p_prev.y) * frame2sec);
        break;
    }

    return output;
}
