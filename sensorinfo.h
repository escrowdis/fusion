#ifndef SENSORINFO_H
#define SENSORINFO_H

#include <opencv2/opencv.hpp>

// sensor base
#include "sensorbase.h"

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// Radar ESR controller
#include "radarcontroller.h"

namespace SENSOR {
enum {
    SV,
    RADAR,
    LRF
};
}

namespace VEHICLE {
enum {
    CART,
    CAR
};
}

class SensorInfo : public QObject, public SensorBase
{
    Q_OBJECT

public:
    SensorInfo();

    ~SensorInfo();

    stereo_vision* sv;

    RadarController* rc;

    lrf_controller* lrf;

    // Sensors' information ===
    bool svDataExec();

    bool radarDataExec();

    bool lrfDataExec();

    void lrfBufExec();
    // Sensors' information === End

    // Fusion =================
    QPixmap pic_sv, pic_radar;

    cv::Mat fused_topview_BG;

    cv::Mat fused_topview;

    void chooseVehicle(int vehicle_type);

    void initialFusedTopView(int range_pixel);

    void updateFusedTopView();

    void drawFusedTopView(bool fg_sv, bool fg_sv_each, bool fg_radar);

    void dataFused();

    void zoomOutFusedTopView();

    void zoomInFusedTopView();
    // Fusion ================= End

private:
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

    // information of objects detected by different sensors on fused topview
    struct sensorInformation {
        SensorBase::sensorLocation location;

        cv::Point pos_pixel;                        // (pixel)

        float angle_half_fov;                       // the half fov for displaying (deg)

        cv::Scalar color;                           // sensor's color on the topview

        cv::Scalar color_fov;

        sensorInformation() {
            pos_pixel = cv::Point(-1, -1);

            angle_half_fov = -1.0;

            color = cv::Scalar(0, 0, 0);

            color_fov = cv::Scalar(200, 200, 200, 255);
        }
    };

    sensorInformation *sensors;

    struct vehicleInfo {
        cv::Point VCP;                              // fused topview vehicle current position (pixel)

        int width;                                  // average vehicle's size (cm)

        int length;                                 // (cm)

        int head_pos;                               // (cm)

        cv::Rect rect;

        cv::Scalar color;                           // vehicle color
    } vehicle;

    int gap = 500;

    PC rangeWorldCalculation(cv::Point sensor_pos, PC pc);
    double pointTransformTopView(cv::Point sensor_pos, double range, double angle, cv::Point *output);
    double pointTransformTopView(cv::Point sensor_pos, double range, double angle, cv::Point *output, cv::Rect rect_in, cv::Rect *rect);
    // Fusion ================= End

signals:
    void updateGUI(cv::Mat *fused_topview, cv::Mat *img_detected_display);
};

#endif // SENSORINFO_H
