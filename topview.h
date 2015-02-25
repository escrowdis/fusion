#ifndef TOPVIEW_H
#define TOPVIEW_H

#include "debug_info.h"

#include <stack>

#include <QImage>

#include "opencv2/opencv.hpp"

class TopView
{
public:
    TopView(int obj_nums, int thresh_free_space, int min_distance, int max_distance, float view_angle, int chord_length, int display_row, int display_col, int grid_row, int grid_col);

    ~TopView();

    // malloc and set the grid coordinates
    void initialTopView();

    void drawTopViewLines(int rows_interval, int cols_interval, bool fg_tag);

    bool isInitializedTopView() {return fg_topview;}

    virtual void pointProjectTopView() {}

    void changeParams(float view_angle, int chord_length);

    cv::Point pointT(cv::Point src);    // transformed point for display

    int min_distance;

    int max_distance;

    cv::Mat topview;                // topview on label

    cv::Mat topview_BG;

    QImage *color_table;            // psuedo-color table //**// wanna make it static

protected:
    void resetTopView();

    int img_col;                    // topview size on label

    int img_col_half;

    int img_row;

    float ratio_row;

    float ratio_col;

    int display_row;

    int display_col;

    struct blobNode
    {
        int pts_num;    // pixels in the cell

        bool labeled;   // filtered object

        int obj_label;  // object label number

        blobNode() {
            obj_label = -1;
            labeled = false;
            pts_num = 0;
        }
    };

    blobNode** grid_map;            // Storing pixels' information into cells

    int obj_nums;

    struct objInformation
    {
        cv::Point tl;   // Top left

        cv::Point br;   // Bottom right

        int X;

        int Y;

        int Z;

        int pts_num;

        objInformation() {
            tl = cv::Point(-1, -1);
            br = cv::Point(-1, -1);
            X = -1;
            Y = -1;
            Z = -1;
            pts_num = 0;
        }
    };

    objInformation* objects;           // filtered objects

    cv::Point** img_grid;           // topview background cell points

    float k;                        // length of interval

    float c;                        //**// the number of adjacent image columns grouped into a polar slice

    float view_angle;               // the view angle

    int chord_length;               // the chord length of stereo vision

    int thresh_free_space;          // check whether the cell is satisfied as an object.
                                    // Each cell containing more than this value is consider as an object.

private:
    void releaseTopView();

    bool fg_topview;                // whether topview is initialized

    // psuedo-color table
    void pseudoColorTable();

    // color
    cv::Scalar color_BG;

    cv::Scalar color_tag;

    cv::Scalar color_line;
};

#endif // TOPVIEW_H
