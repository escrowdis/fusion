#ifndef TOPVIEW_H
#define TOPVIEW_H

#include "debug_info.h"

#include <QImage>

#include "opencv2/opencv.hpp"

class TopView
{
public:
    TopView(int thresh_free_space, int min_distance, int max_distance, float view_angle, int chord_length, int display_row, int display_col, int grid_row, int grid_col);

    ~TopView();

    // malloc and set the grid coordinates
    void initialTopView();

    void drawTopViewLines(int rows_interval, int cols_interval, bool fg_tag);

    bool isInitializedTopView() {return fg_topview;}

    virtual void pointProjectTopView() {}

    void changeParams(float view_angle, int chord_length);

    cv::Point pointT(cv::Point src);    // transformed point from grid map to topview for display

    int min_distance;                   // (cm)

    int max_distance;                   // (cm)

    float view_angle;                   // the view angle (degree)

    float c;                            // ratio of image to grid_map (the number of adjacent image columns grouped into a polar slice)

    cv::Mat topview;                    // topview on label

    cv::Mat topview_BG;

    QImage *color_table;                // psuedo-color table

    // color
    cv::Scalar color_BG;

    cv::Scalar color_tag;

    cv::Scalar color_line;

protected:
    void resetTopView();

    int thick_polygon;

    // grid map size
    int img_col;

    int img_col_half;

    int img_row;

    // ratio for transforming from grid map to topview
    float ratio_row;                    // ratio of display_row to max_distance

    float ratio_col;                    // ratio of display_col to chord_length

    // topview size
    int display_row;

    int display_col;

    struct blobNode
    {
        // GRIDMAP -----------
        int pts_num;                    // pixels in the cell

        int obj_label;                  // object label number

        int avg_Z;                      // average depth of this cell

        int avg_X;

        int avg_Y;

        blobNode() {
            obj_label = -1;
            pts_num = 0;
            avg_Z = 0;
            avg_X = 0;
            avg_Y = 0;
        }
    };

    blobNode** grid_map;                // Storing pixels' information into cells

    cv::Point** img_grid;               // topview background cell points

    float k;                            // length of interval

    int chord_length;                   // the chord length of stereo vision

    int chord_length_min;               // the chord length at min_distance

    int thresh_free_space;              // check whether the cell is satisfied as an object.
                                        // Each cell containing more than this value is consider as an object.

private:
    void releaseTopView();

    bool fg_topview;                // whether topview is initialized

    // psuedo-color table
    void pseudoColorTable();
};

#endif // TOPVIEW_H
