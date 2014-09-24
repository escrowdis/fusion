#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include <QString>
#include <QDebug>
#include <QThread>

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>


// Laser Range Finder Controller
class lrf_controller
{
public:
    lrf_controller();

    bool open(QString comPortIn, int baudRateIn);

    bool isOpen() { return serial->isOpen(); }

    void acquireData(double* data);

private:
    QSerialPort *serial;

    void request();
};

#endif // LRF_CONTROLLER_H
