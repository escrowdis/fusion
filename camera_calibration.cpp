#include "camera_calibration.h"

#define CAP_WIDTH 480
#define CAP_HEIGHT 640

camera_calibration::camera_calibration()
{
}

void camera_calibration::CameraCalibration(bool ShowPts, cv::Size SizePattern, cv::Size2f SizeGrid, std::vector<std::string> files)
{
    // Parameter check
    patternSize = SizePattern;
    gridSize = SizeGrid;
    LoadFile(files);

    FindGetCornerPts(ShowPts, files);
    GetSupposePts();
    cv::Size imageSize = cv::Size(CAP_WIDTH, CAP_HEIGHT);

    intrinsicMat = cv::Mat::eye(3, 3, CV_64F);
    distortionMat = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rVec, tVec;
    cv::Mat map1, map2;

    cv::calibrateCamera(supposePts, PatternPts, imageSize, intrinsicMat, distortionMat,
                        rVec, tVec, cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    cv::initUndistortRectifyMap(intrinsicMat, distortionMat, cv::Mat(), cv::getOptimalNewCameraMatrix(intrinsicMat, distortionMat, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);

}

void camera_calibration::LoadFile(std::vector<std::string> files)
{
    imgSrc.clear();
    for (int i = 0; i < files.size(); i++) {
        cv::Mat img = cv::imread(files[i]);
        imgSrc.push_back(img);
    }

}

void camera_calibration::FindGetCornerPts(bool ShowPts, std::vector<std::string> files)
{
    // Find corner on chessboard
    cv::Mat img, img_g, img_r;
    // Find corner
    PatternPts.clear();
    std::vector<cv::Point2f> corners;
    for (int i = 0; i < imgSrc.size(); i++) {
        img = imgSrc[i];
        cv::cvtColor(img, img_g, cv::COLOR_BGR2GRAY);
        bool PatternFound = cv::findChessboardCorners(img_g, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        img.copyTo(img_r);
        if (PatternFound) {
            cv::cornerSubPix(img_g, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            PatternPts.push_back(corners);
            cv::drawChessboardCorners(img_r, patternSize, cv::Mat(corners), PatternFound);
        }
        else
            cv::putText(img_r, "No corner's been found.", cv::Point(img_r.cols/2 - 160, img_r.rows/2), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255), 1);

        if (ShowPts) {
            cv::imshow("Corner", img_r);
            char c  = cv::waitKey();
            if (c == 27)
                break;
        }
    }
    cv::destroyAllWindows();
}

// Get supposed points
void camera_calibration::GetSupposePts()
{
    supposePts.clear();
    for (int k = 0; k < PatternPts.size(); k++) {
        std::vector<cv::Point3f> buf;
        buf.reserve(patternSize.height * patternSize.width);
        for (int i = 0; i < patternSize.height; i++) {
            for (int j = 0; j < patternSize.width; j++) {
                buf.push_back(cv::Point3f(float(j * gridSize.width), float(i * gridSize.height), 0.0));
            }
        }
        supposePts.push_back(buf);
    }

}

// Save Intrinsic
void camera_calibration::SaveIntrinsic(std::string &folder, std::string &filename)
{
    cv::FileStorage fo(folder + filename, cv::FileStorage::WRITE);
    if (!fo.isOpened()) {
        std::cout<<"Can't save file\n" << folder + filename << std::endl;
        return;
    }

    fo << "camera_matrix" << intrinsicMat;
    fo << "distortion_coefficients" << distortionMat;
    fo.release();
}

// Display result
void camera_calibration::DisplayUndistortedImg(bool ShowPts)
{
    if (imgSrc.empty()) {
        std::cout << "Some files go wrong" << std::endl;
        return;
    }
    int imgW = imgSrc[0].cols;
    int imgH = imgSrc[0].rows;
    cv::Mat tmp;
    for (int i = 0; i < imgSrc.size(); i++) {
        cv::Mat imgUndistortion;
        cv::Mat imgDisplay = cv::Mat(imgH, 2*imgW, CV_8UC3);
        if (ShowPts == true) {
            tmp = imgSrc[i].clone();
            cv::drawChessboardCorners(tmp, patternSize, PatternPts[i], true);
            tmp.copyTo(imgDisplay(cv::Rect(0, 0, imgW, imgH)));
        }
        else
            imgSrc[i].copyTo(imgDisplay(cv::Rect(0, 0, imgW, imgH)));
        cv::undistort(imgSrc[i], imgUndistortion, intrinsicMat, distortionMat);
        if (ShowPts == true) {
            cv::Mat img, img_g, img_r;
            img = imgUndistortion.clone();
            // Find corner
            std::vector<cv::Point2f> corners;
            cv::cvtColor(img, img_g, cv::COLOR_BGR2GRAY);
            bool PatternFound = cv::findChessboardCorners(img_g, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

            img.copyTo(img_r);
            if (PatternFound) {
                cv::cornerSubPix(img_g, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                cv::drawChessboardCorners(img, patternSize, cv::Mat(corners), PatternFound);
            }
            img.copyTo(imgDisplay(cv::Rect(imgW, 0, imgW, imgH)));
        }
        else
            imgUndistortion.copyTo(imgDisplay(cv::Rect(imgW, 0, imgW, imgH)));

        cv::imshow("Distortion -> Undistortion image", imgDisplay);
        char c = cv::waitKey(0);
        if (c == 27)
            break;
    }

    cv::destroyAllWindows();
}

