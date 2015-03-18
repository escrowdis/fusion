#-------------------------------------------------
#
# Project created by QtCreator 2014-09-23T10:08:18
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Fusion
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
        lrf_controller.cpp \
    stereo_vision.cpp \
    calibrationform.cpp \
    camera_calibration.cpp \
    stereomatchparamform.cpp \
    radarcontroller.cpp \
    topview.cpp

HEADERS  += mainwindow.h \
            lrf_controller.h \
    stereo_vision.h \
    debug_info.h \
    calibrationform.h \
    camera_calibration.h \
    stereomatchparamform.h \
    radarcontroller.h \
    topview.h

FORMS    += mainwindow.ui \
    calibrationform.ui \
    stereomatchparamform.ui

RESOURCES += \
    icon.qrc

RC_ICONS = icon\\car.ico

OpenCV_Lib = D:/OpenCV/opencv3beta_cuda/x86/vc12/lib

CAN_Lib = $$quote(C:\Program Files (x86)\Kvaser\Canlib\Lib\MS)

INCLUDEPATH +=  D:/OpenCV/opencv3beta_cuda/include \
                $$quote(C:\Program Files (x86)\Kvaser\Canlib\INC)   # CANLib

Release: LIBS +=  $$OpenCV_Lib/opencv_calib3d300.lib \
                    $$OpenCV_Lib/opencv_core300.lib \
                    $$OpenCV_Lib/opencv_cuda300.lib  \
                    $$OpenCV_Lib/opencv_cudaarithm300.lib  \
                    $$OpenCV_Lib/opencv_cudabgsegm300.lib  \
                    $$OpenCV_Lib/opencv_cudacodec300.lib  \
                    $$OpenCV_Lib/opencv_cudafeatures2d300.lib  \
                    $$OpenCV_Lib/opencv_cudafilters300.lib  \
                    $$OpenCV_Lib/opencv_cudaimgproc300.lib  \
                    $$OpenCV_Lib/opencv_cudalegacy300.lib  \
                    $$OpenCV_Lib/opencv_cudaoptflow300.lib\
                    $$OpenCV_Lib/opencv_cudastereo300.lib  \
                    $$OpenCV_Lib/opencv_cudawarping300.lib \
                    $$OpenCV_Lib/opencv_cudev300.lib  \
                    $$OpenCV_Lib/opencv_features2d300.lib  \
                    $$OpenCV_Lib/opencv_flann300.lib  \
                    $$OpenCV_Lib/opencv_highgui300.lib  \
                    $$OpenCV_Lib/opencv_imgcodecs300.lib  \
                    $$OpenCV_Lib/opencv_imgproc300.lib  \
                    $$OpenCV_Lib/opencv_ml300.lib  \
                    $$OpenCV_Lib/opencv_objdetect300.lib  \
                    $$OpenCV_Lib/opencv_photo300.lib  \
                    $$OpenCV_Lib/opencv_shape300.lib  \
                    $$OpenCV_Lib/opencv_stitching300.lib  \
                    $$OpenCV_Lib/opencv_superres300.lib  \
                    $$OpenCV_Lib/opencv_ts300.lib  \
                    $$OpenCV_Lib/opencv_video300.lib  \
                    $$OpenCV_Lib/opencv_videoio300.lib  \
                    $$OpenCV_Lib/opencv_videostab300.lib  \
                    $$OpenCV_Lib/opencv_viz300.lib\
                  $$CAN_Lib/canlib32.lib\
                  $$CAN_Lib/j1587lib.lib\
                  $$CAN_Lib/j2534api.lib\
                  $$CAN_Lib/kvaDbLib.lib\
                  $$CAN_Lib/kvj2534.lib\
                  $$CAN_Lib/kvrlib.lib\
                  $$CAN_Lib/linlib.lib\
                  $$CAN_Lib/sing32.lib\
                  $$CAN_Lib/vcand32.lib

Debug: LIBS +=    $$OpenCV_Lib/opencv_calib3d300d.lib \
                   $$OpenCV_Lib/opencv_core300d.lib \
                   $$OpenCV_Lib/opencv_cuda300d.lib  \
                   $$OpenCV_Lib/opencv_cudaarithm300d.lib  \
                   $$OpenCV_Lib/opencv_cudabgsegm300d.lib  \
                   $$OpenCV_Lib/opencv_cudacodec300d.lib  \
                   $$OpenCV_Lib/opencv_cudafeatures2d300d.lib  \
                   $$OpenCV_Lib/opencv_cudafilters300d.lib  \
                   $$OpenCV_Lib/opencv_cudaimgproc300d.lib  \
                   $$OpenCV_Lib/opencv_cudalegacy300d.lib  \
                   $$OpenCV_Lib/opencv_cudaoptflow300d.lib  \
                   $$OpenCV_Lib/opencv_cudastereo300d.lib  \
                   $$OpenCV_Lib/opencv_cudawarping300d.lib  \
                   $$OpenCV_Lib/opencv_cudev300d.lib  \
                   $$OpenCV_Lib/opencv_features2d300d.lib  \
                   $$OpenCV_Lib/opencv_flann300d.lib  \
                   $$OpenCV_Lib/opencv_highgui300d.lib  \
                   $$OpenCV_Lib/opencv_imgcodecs300d.lib  \
                   $$OpenCV_Lib/opencv_imgproc300d.lib  \
                   $$OpenCV_Lib/opencv_ml300d.lib  \
                   $$OpenCV_Lib/opencv_objdetect300d.lib  \
                   $$OpenCV_Lib/opencv_photo300d.lib  \
                   $$OpenCV_Lib/opencv_shape300d.lib  \
                   $$OpenCV_Lib/opencv_stitching300d.lib  \
                   $$OpenCV_Lib/opencv_superres300d.lib  \
                   $$OpenCV_Lib/opencv_ts300d.lib  \
                   $$OpenCV_Lib/opencv_video300d.lib  \
                   $$OpenCV_Lib/opencv_videoio300d.lib  \
                   $$OpenCV_Lib/opencv_videostab300d.lib  \
                   $$OpenCV_Lib/opencv_viz300d.lib\
                  $$CAN_Lib/canlib32.lib\
                  $$CAN_Lib/j1587lib.lib\
                  $$CAN_Lib/j2534api.lib\
                  $$CAN_Lib/kvaDbLib.lib\
                  $$CAN_Lib/kvj2534.lib\
                  $$CAN_Lib/kvrlib.lib\
                  $$CAN_Lib/linlib.lib\
                  $$CAN_Lib/sing32.lib\
                  $$CAN_Lib/vcand32.lib
