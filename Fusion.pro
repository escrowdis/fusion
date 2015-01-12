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

OpenCV_Lib = D:/OpenCV/opencv3alpha/x86/vc12/lib

CAN_Lib = $$quote(C:\Program Files (x86)\Kvaser\Canlib\Lib\MS)

INCLUDEPATH +=  D:/OpenCV/opencv3alpha/include\
                $$quote(C:\Program Files (x86)\Kvaser\Canlib\INC)   # CANLib

Release: LIBS +=  $$OpenCV_Lib/opencv_ts300.lib\
                  $$OpenCV_Lib/opencv_world300.lib\
                  $$CAN_Lib/canlib32.lib\
                  $$CAN_Lib/j1587lib.lib\
                  $$CAN_Lib/j2534api.lib\
                  $$CAN_Lib/kvaDbLib.lib\
                  $$CAN_Lib/kvj2534.lib\
                  $$CAN_Lib/kvrlib.lib\
                  $$CAN_Lib/linlib.lib\
                  $$CAN_Lib/sing32.lib\
                  $$CAN_Lib/vcand32.lib

Debug: LIBS +=    $$OpenCV_Lib/opencv_ts300d.lib\
                  $$OpenCV_Lib/opencv_world300d.lib\
                  $$CAN_Lib/canlib32.lib\
                  $$CAN_Lib/j1587lib.lib\
                  $$CAN_Lib/j2534api.lib\
                  $$CAN_Lib/kvaDbLib.lib\
                  $$CAN_Lib/kvj2534.lib\
                  $$CAN_Lib/kvrlib.lib\
                  $$CAN_Lib/linlib.lib\
                  $$CAN_Lib/sing32.lib\
                  $$CAN_Lib/vcand32.lib
