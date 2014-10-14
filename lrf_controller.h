#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include "debug_info.h"

#include <QString>
#include <QTime>

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>

#define LENGTH_DATA 361
#define LENGTH_RAW_DATA 733
#define LENGTH_HEADER 8
#define RESOLUTION 0.5

const uchar header_data[] = {0x06, 0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};

const char request_baud_rate_to_9600[] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x42, 0x52, 0x08};
const char request_data_once[] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
const char request_data_continuous[] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};
const char request_data_stop[] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x35, 0x08};

const uchar ACK_baudrate[] = {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};

// Laser Range Finder Controller
class lrf_controller
{
public:
    lrf_controller();

    ~lrf_controller();

    bool open(QString comPortIn, int baudRateIn);

    bool isOpen() { return serial->isOpen(); }

    bool acquireData(double *data);

    bool close();

private:
    QSerialPort *serial;

    enum HEADER_TYPE{
        BAUDRATE,
        DATA
    };

    enum CAPTURE_MODE {
        ONCE,
        CONTINUOUS
    };

    void requestData(int mode = CAPTURE_MODE::ONCE);

    QSerialPort::BaudRate baudRate;

    unsigned char data_raw[LENGTH_RAW_DATA - LENGTH_HEADER];

    ushort doCRC(const QByteArray &data);

    uchar checkACK(const uchar *msg_ack);

    bool checkHeader(QByteArray &data, int header_type);
};

#endif // LRF_CONTROLLER_H
