#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include "debug_info.h"

#include <QThread>

#include <opencv2/opencv.hpp>

class camera_calibration : public QThread
{
    Q_OBJECT
public:
    explicit camera_calibration(QObject *parent = 0);

    ~camera_calibration();

    void CameraCalibration(bool ShowPts, cv::Size SizePattern, cv::Size2f SizeGrid, std::vector<std::string> files);

    void FindGetCornerPts(bool ShowPts, std::vector<std::string> files);

    void SaveIntrinsic(std::string &folder, std::string &filename);

    void DisplayUndistortedImg(bool ShowPts);

    void start();

    void stop();

private:

    void LoadFile(std::vector<std::string> files);

    void GetSupposePts();

    std::vector<cv::Mat> imgSrc;

    // Record images for calibration
    std::vector<std::vector<cv::Point3f> > supposePts;
    std::vector<std::vector<cv::Point2f> > PatternPts;
    cv::Size patternSize;       // Number of corner
    cv::Size2f gridSize;          // Size of grid

    // Output parameter
    cv::Mat  intrinsicMat, distortionMat;

    bool fg_run;
    bool fg_end;

protected:
    void run();

signals:
    void saveImage();
};

#endif // CAMERA_CALIBRATION_H
