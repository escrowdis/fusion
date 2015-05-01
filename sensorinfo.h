#ifndef SENSORINFO_H
#define SENSORINFO_H

#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// Radar ESR controller
#include "radarcontroller.h"

class SensorInfo
{
public:
    SensorInfo();

    ~SensorInfo();

    RadarController* rc;

    stereo_vision* sv;

    lrf_controller* lrf;


    // Fusion =================
    // topview
    int detection_range_pixel;                      // fused topview radius (pixel)

    float detection_range;                          // real detection range radius (cm)

    float ratio;                                    // scaling ratio of real to fused topview (cm -> pixel)

    float max_detection_range;                      // max. detection range in all sensors (cm)

    float min_detection_range;                      // min. detection range in all sensors (cm)

    int thickness;                                  // object params on topview

    int font;

    int font_size;

    int font_thickness;

    QPixmap pic_sv, pic_radar;

    // information of objects detected by different sensors on fused topview
    struct sensorInformation {
        cv::Point pos;                              // sensor's position based on the center of vehicle
                                                    // (pixel) forward is x+, lateral right is y+.

        cv::Point pos_pixel;

        cv::Scalar color;                           // sensor's color on the topview

        float angle;                                // the half fov for displaying (deg)

        cv::Scalar color_fov;

        sensorInformation() {
            pos = cv::Point(-1, -1);

            color = cv::Scalar(0, 0, 0);

            angle = -1.0;
        }
    };

    sensorInformation *sensors;
    // sensors:
    // [0] = stereo vision
    // [1] = radar ESR
    // [2] = laser rangefinder

    struct vehicleInfo {
        cv::Point VCP;                              // fused topview vehicle current position (pixel)

        int width;                                  // average vehicle's size (cm)

        int length;                                 // (cm)

        int head_pos;                               // (cm)

        cv::Rect rect;

        cv::Scalar color;                           // vehicle color
    } vehicle;

    cv::Mat fused_topview_BG;

    cv::Mat fused_topview;

    void VehicleCart();

    void VehicleCar();

    void initialFusedTopView(int range_pixel);

    void updateFusedTopView();

    void drawFusedTopView(bool fg_sv, bool fg_sv_each, bool fg_radar);

    float pointTransformTopView(cv::Point sensor_pos, float range, float angle, cv::Point *output);
    float pointTransformTopView(cv::Point sensor_pos, float range, float angle, cv::Point *output, cv::Rect rect_in, cv::Rect *rect);

    void dataFused();

    int gap = 500;

    void zoomOutFusedTopView();

    void zoomInFusedTopView();
    // ======================== End
};

#endif // SENSORINFO_H
