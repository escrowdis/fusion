#ifndef OBJECTMATCHING_H
#define OBJECTMATCHING_H

#include "../debug_info.h"

#include "opencv2/opencv.hpp"

#include "sensorbase.h"

namespace MATCH_TYPE {
enum {
    RANGE_ONLY,
    RANGE_BHA,
};
}

class ObjectMatching
{
public:
    ObjectMatching();

    ~ObjectMatching();

    void initializeObjectMatching(int size);

    void resetObjMatching();

    void setThresholdBha(double thresh);

    void setErrorThresholdZ(double max_err_z, double max_distance_z);

    void setErrorThresholdX(double thresh_err_x);

    void splitOneOut(int channel, cv::Mat src, cv::Mat *dst);

    virtual void resetMatchedInfo() {}

    virtual void connectMatchedInfo() {}

    virtual void dataMatching() {}

    std::vector<std::pair<int, int> > Matching();

    std::vector<std::pair<int, int> > matchingList; // store result of matching

    // histogram calculation used
    int hist_size;
    const float *hist_ranges;

    // check if there's any object detected
    bool fg_om_existed;

    // amount of detected object
    int om_obj_num;

    struct objectMatchingInfo
    {
        bool labeled;

        int match_type;

        SensorBase::PC pc;

        std::pair<double, double> err_pos;

        cv::Mat img;                    // object's image (cropped)

        cv::Mat H_img;

        cv::Mat H_hist;

        bool fg_Bha_check;

        // IMAGE -------------
        std::pair<int, int> center;     // Center point of object in image (row, col)
    };

    objectMatchingInfo *om;
    objectMatchingInfo *om_prev;

#ifdef debug_info_object_matching_img
    cv::Mat comp, comp_prev;
#endif

private:
    bool fg_initialized = false;

    // Used for SV only
    double thresh_Bha = 0.3; //**// tuned params 20150528
    double max_distance_z = 3000;
    double max_err_z = 500;
    cv::Mat map_thresh_err_z;
    double thresh_err_x = 300;  //**// tuned params 20150528

    // check if there's any object detected
    bool fg_om_prev_existed;

    // amount of objects. Default: obj_nums
    int om_size;
    int om_prev_size;

    // amount of detected object
    int om_prev_obj_num;

    // object's id <-> map_Bha id
    std::vector<int> map_Bha_corr_id_r;
    std::vector<int> map_Bha_corr_id_c;

    void resetMatchingInfo(objectMatchingInfo &src);

    void moveOm(objectMatchingInfo &src, objectMatchingInfo &dst);
};

#endif // OBJECTMATCHING_H
