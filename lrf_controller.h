#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include "debug_info.h"

#include <QString>

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>

#define LENGTH_DATA 361
#define LENGTH_RAW_DATA 733
#define LENGTH_HEADER 7
#define RESOLUTION 0.5

// Laser Range Finder Controller
class lrf_controller
{
public:
    lrf_controller();

    bool open(QString comPortIn, int baudRateIn);

    bool isOpen() { return serial->isOpen(); }

    bool acquireData(double* data);

    bool close();

private:
    QSerialPort *serial;

    void request();

    QSerialPort::BaudRate baudRate;

    unsigned char data_raw[LENGTH_RAW_DATA - LENGTH_HEADER];
};

#endif // LRF_CONTROLLER_H
