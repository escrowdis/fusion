#ifndef TOPVIEW_H
#define TOPVIEW_H

#include "debug_info.h"

#include <QImage>

#include "opencv2/opencv.hpp"

#define MIN_DISTANCE 200 // cm, minimum detection depth
#define MAX_DISTANCE 3000

class TopView
{
public:
    TopView(int thresh_free_space);

    ~TopView();

    // malloc and set the grid coordinates
    void initialTopView();

    void drawTopViewLines(int rows_interval, int cols_interval);

    bool isInitializedTopView() {return fg_topview;}

    virtual void pointProjectTopView() {}

    cv::Mat topview;                // topview on label

    cv::Mat topview_BG;

    QImage *color_table;            // psuedo-color table //**// wanna make it static

protected:
    void resetTopView();

    int img_col;                    // topview size on label

    int img_col_half;

    int img_row;

    int** grid_map;                 // Storing pixels into cells

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
};

#endif // TOPVIEW_H
