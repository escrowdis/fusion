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
    camera_calibration.cpp

HEADERS  += mainwindow.h \
            lrf_controller.h \
    stereo_vision.h \
    debug_info.h \
    calibrationform.h \
    camera_calibration.h

FORMS    += mainwindow.ui \
    calibrationform.ui

OpenCV_Lib = D:/OpenCV/opencv3alpha/x86/vc12/lib

INCLUDEPATH +=  D:/OpenCV/opencv3alpha/include

Release: LIBS +=  $$OpenCV_Lib/opencv_ts300.lib\
                  $$OpenCV_Lib/opencv_world300.lib

Debug: LIBS +=    $$OpenCV_Lib/opencv_ts300d.lib\
                  $$OpenCV_Lib/opencv_world300d.lib
