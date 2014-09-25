#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include <QString>
#include <QDebug>
#include <QThread>

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>

#define length_data 361
#define length_raw_data 733
#define length_header 11
#define resolution 0.5



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

    unsigned char data_raw[length_raw_data - length_header];
};

#endif // LRF_CONTROLLER_H
