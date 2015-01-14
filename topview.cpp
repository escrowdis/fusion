#include "topview.h"

TopView::TopView(int thresh_free_space)
{
    fg_topview = false;

    img_col = 100;

    img_col_half = img_col / 2;

    img_row = 125;

    c = 6.4;

    k = 0.02;

    this->thresh_free_space = thresh_free_space;

    //**// diff cam, diff fov
    view_angle = 19.8;

    chord_length = 1080;

    initialTopView();

    pseudoColorTable();
}

TopView::~TopView()
{
    releaseTopView();

    delete color_table;
}

void TopView::releaseTopView()
{
    if (fg_topview) {
        for (int i = 0; i < (img_row + 1); i++)
            delete[] img_grid[i];
        delete[] img_grid;

        for (int i = 0; i < (img_row + 1); i++)
            delete[] grid_map[i];
        delete[] grid_map;

        fg_topview = false;
    }
}

void TopView::pseudoColorTable()
{
    // ==== Produce pseudo-color table
    color_table = new QImage(MAX_DISTANCE - MIN_DISTANCE, 12, QImage::Format_RGB888) ;

    float hue_start_angle = 0.0;
    float hue_end_angle = 240.0;

    float h = 0.0;
    float s = 1.0;
    float v = 255.0;
    float step = (hue_end_angle - hue_start_angle) / (MAX_DISTANCE - MIN_DISTANCE);

    float f;
    int hi, p, q, t;

    for (int r = 0; r < color_table->height(); r++) {
        uchar* ptr = color_table->scanLine(r);
        for (int c =0; c < color_table->width(); c++) {
            h = c * step;
            hi = (int)(h / 60.0) % 6;
            f = h / 60.0 - hi;
            p = v * (1 - s);
            q = v * (1 - f * s);
            t = v * (1 - (1 - f) * s);

            switch (hi) {
            case 0:
                ptr[3 * c + 0] = v;
                ptr[3 * c + 1] = t;
                ptr[3 * c + 2] = p;
                break;
            case 1:
                ptr[3 * c + 0] = q;
                ptr[3 * c + 1] = v;
                ptr[3 * c + 2] = p;
                break;
            case 2:
                ptr[3 * c + 0] = p;
                ptr[3 * c + 1] = v;
                ptr[3 * c + 2] = t;
                break;
            case 3:
                ptr[3 * c + 0] = p;
                ptr[3 * c + 1] = q;
                ptr[3 * c + 2] = v;
                break;
            case 4:
                ptr[3 * c + 0] = t;
                ptr[3 * c + 1] = p;
                ptr[3 * c + 2] = v;
                break;
            case 5:
                ptr[3 * c + 0] = v;
                ptr[3 * c + 1] = p;
                ptr[3 * c + 2] = q;
                break;
            }
        }
    }
}

void TopView::initialTopView()
{
    if (!fg_topview) {
        img_grid = new cv::Point * [img_row + 1];
        for (int r = 0; r < img_row + 1; r++)
            img_grid[r] = new cv::Point[img_col + 1];

        grid_map = new int * [img_row];
        for (int r = 0; r< img_row; r++)
            grid_map[r] = new int[img_col];

        resetTopView();
    }

    for (int r = 0; r < img_row + 1; r++) {
        for (int c = 0; c < img_col_half + 1; c++) {
            int z = MIN_DISTANCE * pow(1.0 + k, r);
            int x = z * tan(0.5 * view_angle * CV_PI / 180.0 * c / img_col_half);

            x > 0.5 * chord_length ? 0.5 * chord_length : x;

            if (c == 0) {
                img_grid[r][img_col_half] = cv::Point(0.5 * chord_length - x, MAX_DISTANCE - z);
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r][img_col_half + 1], 3, cv::Scalar(0, 255, 0, 255), 5, 8, 0);
#endif
            }
            else {
                img_grid[r][img_col_half - c] = cv::Point(0.5 * chord_length - x, MAX_DISTANCE - z);
                img_grid[r][img_col_half + c] = cv::Point(0.5 * chord_length + x, MAX_DISTANCE - z);
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r][img_col_half + 1 - c], 3, cv::Scalar(255, 0, 0, 255), 5, 8, 0);
                cv::circle(topview, img_grid[r][img_col_half + 1 + c], 3, cv::Scalar(0, 0, 255, 255), 5, 8, 0);
#endif
            }
            if (r == img_row) {
                x = MAX_DISTANCE * tan(0.5 * view_angle * CV_PI / 180.0 * c / img_col_half);
                img_grid[r][img_col_half - c] = cv::Point(0.5 * chord_length - x, 0);
                img_grid[r][img_col_half + c] = cv::Point(0.5 * chord_length + x, 0);
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r + 1][img_col_half + 1 - c], 3, cv::Scalar(255, 255, 0, 255), 5, 8, 0);
                cv::circle(topview, img_grid[r + 1][img_col_half + 1 + c], 3, cv::Scalar(0, 255, 255, 255), 5, 8, 0);
#endif
            }

        }
    }

#ifdef debug_info_sv_topview
    cv::Mat topview_re;
    cv::resize(topview, topview_re, cv::Size(400, 800));
    cv::imshow("hi", topview_re);
#endif

    fg_topview = true;
}

void TopView::drawTopViewLines(int rows_interval, int cols_interval)
{
    topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    // Background color
    cv::Point pts[4];
    pts[0] = img_grid[0][0];
    pts[1] = img_grid[0][img_col];
    pts[2] = img_grid[img_row][img_col];
    pts[3] = img_grid[img_row][0];
    cv::fillConvexPoly(topview_BG, pts, 4, cv::Scalar(161, 158, 149, 255), 8, 0);

    // draw lines
    for (int r = 0; r < img_row + 1; r += rows_interval) {
        cv::line(topview_BG, img_grid[r][0], img_grid[r][img_col], cv::Scalar(0, 255, 0, 255), 10, 8, 0);
    }
    for (int c = 0; c < img_col_half + 1; c += cols_interval) {
#ifdef debug_info_sv_topview
        std::cout<<img_col_half + 1 - c<<" ";
#endif
        cv::line(topview_BG, img_grid[0][img_col_half - c], img_grid[img_row][img_col_half - c], cv::Scalar(0, 255, 0, 255), 10, 8, 0);
        cv::line(topview_BG, img_grid[0][img_col_half + c], img_grid[img_row][img_col_half + c], cv::Scalar(0, 255, 0, 255), 10, 8, 0);
    }
#ifdef debug_info_sv_topview
    std::cout<<std::endl;
#endif
}

void TopView::resetTopView()
{
    topview.setTo(cv::Scalar(0, 0, 0, 0));

    for (int r = 0; r < img_row; r++)
        for (int c = 0; c < img_col; c++)
            grid_map[r][c] = 0;

    // data mark is reset in objectProjectTopView
}
