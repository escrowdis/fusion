#ifndef SENSORINFO_H
#define SENSORINFO_H

#include <opencv2/opencv.hpp>

#include <QReadWriteLock>
extern QReadWriteLock lock_ot_fused;

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// Radar ESR controller
#include "radarcontroller.h"

// Object Recognition
#include "objectrecognition.h"

// object matching
#include "objectTracking/objectmatching.h"

// Object tracking
#include "objectTracking/objecttracking.h"

// Collision avoidance
#include "collisionAvoidance/collisionavoidance.h"

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
    CAR,
    TRACTOR
};
}

namespace GUI {
enum {
    APPROACHING,
    LEFT_ONCOMING,
    RIGHT_ONCOMING
};
}

//! SensorInfor class
//!
//! \brief This class deals with all the sensors' information and the fused result.
//!
//! fusion function:\n
//!     sigma(SV, X) = p1 * D + p2\n sigma(RADAR, X) = u1 * exp(u2 * D)\n
//!     sigma(SV, Z) = u3 * exp(u4 * D)\n sigma(RADAR,Z) = p3 * D + p4
//!
class SensorInfo : public QObject, private ObjectMatching
{
    Q_OBJECT

public:
    SensorInfo();

    ~SensorInfo();

    stereo_vision* sv;                                  ///< pointer of class stereo_vision

    RadarController* rc;                                ///< pointer of class RadarController

    lrf_controller* lrf;                                ///< pointer of class lrf_controller

    ObjectTracking::objectTrackingInfo *data_fused;     ///< fused information (single)

    void resetTrackingInfo();                           ///< clear std::vector<objectTracking> ti

    ObjectTracking* ot_fused;                           ///< tracking fused data (continuous)

    CollisionAvoidance* ca;                             ///< pointer of class CollisionAvoidance

    // Status =================
    bool fg_proc;                       ///< wether fusion function is processing
    bool fg_sv;                         ///< check if the sv is processing and need to be processed in fusion function
    bool fg_sv_each_pixel;              ///< plot all the pixel information in the fused topview or not
    bool fg_radar;                      ///< check if the radar is processing and need to be processed in fusion function
    bool fg_data_update;                ///< check status of sensors is processing or not
    bool fg_fusion;                     ///< check for run fusion function
    bool fg_ot;                         ///< check for object tracking
    bool fg_ot_trajectory;              ///< check for displaying trajectory on fused topview or not
    bool fg_ot_trajectory_raw;          ///< check for displaying raw traj.
    bool fg_ot_trajectory_kalman;       ///< check for displaying Kalman filtered traj.
    bool fg_ot_vel;                     ///< check for displaying velocity
    bool fg_ot_kf;                      ///< check for running Kalaman filter
    bool fg_ot_pf;                      ///< [unused] for running particle filter
    bool fg_ca_astar;                   ///< check for running A* algo.
    bool fg_ca_vfh;                     ///< check for running VFH algo.
    bool fg_fused_region_display_only;  ///< check for displaying only sensors' overlapped region
    bool fg_display_literature_mode;    ///< displaying mode change button
    // Status ================= End

    // Sensors' information ===
    //!
    //! \brief Retrieving data from stereo vision.
    //! \return int status
    //!
    int svDataExec();

    //!
    //! \brief Retrieving data from radar.
    //! \return int status
    //!
    int radarDataExec();

    //!
    //! \brief Retrieving data from laser rangefinder.
    //! \return bool receive status (not used. 20150525)
    //!
    bool lrfDataExec();

    //!
    //! \brief Retrieving lrf data to buf.
    //!
    void lrfBufExec();
    // Sensors' information === End

    // Fusion =================
    QPixmap pic_sv;             ///< display color of SV sensor
    QPixmap pic_radar;          ///< display color of RADAR sensor

    cv::Mat fused_topview_BG;   ///< background of fused topview

    cv::Mat fused_topview;      ///< foreground of fused topview

    //!
    //! \brief chooseVehicle to modify topview's setting with different vehicles
    //! \param vehicle_type vehicle, tractor and cart.
    //!
    void chooseVehicle(int vehicle_type);

    //!
    //! \brief initialFusedTopView
    //! \param range_pixel radius of fused topview in pixel
    //!
    void initialFusedTopView(int range_pixel);

    //!
    //! \brief updateFusedTopView Update the fused topview w/ different detection range
    //!
    void updateFusedTopView();

    //!
    //! \brief updateFusedData Transform data from PC to display coordinates
    //! \bug Rect from SV won't change as well? 20150623
    //!
    void updateFusedData();

    //!
    //! \brief dataExec Fusion main function
    //!
    void dataExec();

    //!
    //! \brief zoomOutFusedTopView to zoom out the detecting range of fused topview
    //!
    void zoomOutFusedTopView();

    //!
    //! \brief zoomInFusedTopView to zoom in the detecting range of fused topview
    //!
    void zoomInFusedTopView();
    // Fusion ================= End

private:    
    ///< sensor location
    struct sensorLocation {
        cv::Point2f pos;    ///< sensor's position based on the center of vehicle. (cm) forward is x+, lateral right is y+.

        float theta;        ///< Orientation of sensor (degree). Forward is set as zero and counting clockwisely.
    };

    QTime t;                ///< current thread processing time. Used for updating GUI.
    int time_gap;           ///< Threshold elapse of updating GUI

    // Fusion =================
    int size_data_fused;    ///< MAX sotrage amount of object. It's the sum of obj. from SV and RADAR now (2015).

    //!
    //! \brief resetFusion to reset \param data_fused
    //!
    void resetFusion();

    // fusion param (x, z)
    double var_sv[2];                               ///< param. of fusion function of SV
    double var_radar[2];                            ///< param. of fusion function of RADAR
    double var_fused[2];                            ///< param. of fusion function
    double param_p[4];                              ///< curve fitting params for fusion function
    double param_u[4];                              ///< the same as \p param_p

    std::vector<int> cri;                           ///< Array of closest_radar's id
    double U_D;                                     ///< Max distance error (cm)
    double R_sv;                                    ///< Detection radius w/ object's SV center (cm)
    cv::Point2d sv_pos;                             ///< SV's Cartesian position for fusion
    SensorBase::PC radar_mean;                      ///< RADAR's average position if there's more than one retrieved data while a single SV data retrieving.
    cv::Point radar_plot_pt_f;                      ///< RADAR's Cartesian position for fusion
    cv::Point2f radar_vel;                          ///< [unused] RADAR's velocity

    //!
    //! \brief dataProcess to fused topview and fusion information process
    //! \param fg_sv
    //! \param fg_radar
    //!
    void dataProcess(bool fg_sv, bool fg_radar);

    // Object matching ========
    //!
    //! \brief resetMatchedInfo
    //! \param src
    //!
    void resetMatchedInfo(ObjectTracking::objectTrackingInfo &src);

    //!
    //! \brief connectMatchedInfo to hard copy info. from src to dst and then erase src
    //! \param src
    //! \param dst
    //!
    void connectMatchedInfo(ObjectTracking::objectTrackingInfo &src, ObjectTracking::objectTrackingInfo &dst);

    //!
    //! \brief dataMatching to match the adjacent frame object using features.
    //! Object matching: a) Bha. dist. of H color space, b) bias of X & Z of WCS location
    //! Comparison of H color space image using Bhattacharyya distance with Bubble search
    //!
    void dataMatching();

    std::vector<matchedResult> matching_result;     ///< record the result of object matching
    // Object matching ======== End
    cv::RNG rng;                                    ///< rng for traj. color

    //!
    //! \brief dataTracking to use the matching result to sotrage the object info.
    //!
    void dataTracking();

    //!
    //! \brief dataCollisionAvoidance to do the collision avoidance
    //!
    void dataCollisionAvoidance();

    int gui_display_time;                           ///< current thread runtime
    int gui_display_max = 20;                       ///< threshold for updating gui of pre-collision warning

    //!
    //! \brief drawFusedTopView
    //! \param fg_sv
    //! \param fg_radar
    //! \param fg_sv_each
    //! \param fg_ot_trajectory
    //! \param fg_ot_trajectory_raw
    //! \param fg_ot_trajectory_kalman
    //! \param fg_ot_kf
    //! \param fg_fused_region_display_only
    //!
    void drawFusedTopView(bool fg_sv, bool fg_radar, bool fg_sv_each, bool fg_ot_trajectory, bool fg_ot_trajectory_raw, bool fg_ot_trajectory_kalman, bool fg_ot_kf, bool fg_fused_region_display_only);

    int range_precision;                            ///< precision of showing number on the fused topview

    // topview
    int detection_range_pixel;                      ///< fused topview radius (pixel)

    float detection_range;                          ///< real detection range radius (cm)

    float ratio;                                    ///< scaling ratio of real to fused topview (cm -> pixel)

    float max_detection_range;                      ///< Max detection range in all sensors (cm)

    float min_detection_range;                      ///< min detection range in all sensors (cm)

    int thickness;                                  ///< object params on topview

    int font;                                       ///< fused topview display font

    int font_size;                                  ///< fused topview display font size

    int font_thickness;                             ///< fused topview display font thickness

    //!
    //! \brief The sensorInformation struct is the information of objects detected by different sensors. Based on fused topview
    //!
    struct sensorInformation {
        sensorLocation location;                    ///< Sensor location

        cv::Point pos_pixel;                        ///< (pixel)

        float angle_half_fov;                       ///< the half fov for displaying (degree)

        cv::Scalar color;                           ///< color of different sensors on the fused topview

        cv::Scalar color_fov;                       ///< color of fov lines

        sensorInformation() {
            pos_pixel = cv::Point(-1, -1);

            angle_half_fov = -1.0;

            color = cv::Scalar(0, 0, 0);

            color_fov = cv::Scalar(200, 200, 200, 255);
        }
    };

    sensorInformation *sensors;                     ///< sensors' basic information

    //!
    //! \brief The vehicleInfo struct to save the vehicle's basic info.
    //!
    struct vehicleInfo {
        cv::Point VCP;                              ///< fused topview vehicle current position (pixel)

        double width;                               ///< average vehicle's size (cm)

        double length;                              ///< (cm)

        int width_pixel;                            ///< (pixel)

        int length_pixel;                           ///< (pixel)

        double head_pos;                            ///< (cm)

        int head_pos_pixel;                         ///< (pixel)

        cv::Rect rect;                              ///< (pixel)

        cv::Scalar color;                           ///< vehicle color on fused tovpiew
    } vehicle;

    int gap = 500;                                  ///< min change value for changing detection range

    ///
    /// Coordinate transformation used in \link coordinateTransform
    ///
    enum {
        CT_WCS2SCS, ///< from world coordinate sys. to sensor coordinate sys.
        CT_SCS2WCS  ///< from world coor. sys. to sensor coor. sys.
    };

    //!
    //! \brief coordinateTransform. The transformation of coordinates system
    //! \param int type
    //! \param cv::Point sensor_pos
    //! \param SensorBase::PC pc_in
    //! \return SensorBase::PC pc_out
    //!
    SensorBase::PC coordinateTransform(int type, cv::Point sensor_pos, SensorBase::PC pc_in);
    cv::Rect rectOnFusedTopView(cv::Point pt_pixel, cv::Rect rect_in);
    cv::Point point2FusedTopView(cv::Point sensor_pos, SensorBase::PC pc);
    cv::Point point2FusedTopView(SensorBase::PC pc_world);
    cv::Point point2FusedTopView(cv::Point pos_world);
    // Fusion ================= End

private slots:
    //!
    //! \brief drawArrow
    //! \param img
    //! \param pStart arrow's bottom
    //! \param pEnd arrow's head
    //! \param len branch's length
    //! \param alpha angle between main and branch
    //! \param color
    //! \param thickness arrow's thickness
    //! \param lineType
    //!
    void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar& color, int thickness = 1, int lineType = 8);

signals:
    void updateGUI(cv::Mat *fused_topview, cv::Mat *img_detected_display);

    void guiDisplay(int type, bool fg_on);
};

#endif // SENSORINFO_H
