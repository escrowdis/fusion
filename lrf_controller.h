#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include "debug_info.h"

#include <QString>
#include <QTime>
#include <QBuffer>

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>

#define LENGTH_DATA 361
#define LENGTH_RAW_DATA 733
#define LENGTH_HEADER 8
#define LENGTH_ACK 10
#define RESOLUTION 0.5
#define MAX_BUF_SIZE 7330

const char header_data[] =                 {0x06, 0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};
// once mode: header_data pop up
// continuous mode: ACK first and come up w/ header data w/o first uchar 0x06
// SO, header in ONCE and CONTI is 7 and 8 respectively.

const char request_baud_rate_to_9600[] =    {0x02, 0x00, 0x02, 0x00, 0x20, 0x42, 0x52, 0x08};
const char request_data_once[] =            {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
const char request_data_continuous[] =      {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};
const char request_data_stop[] =            {0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x35, 0x08};

const uchar ACK_baudrate[] =                {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};
const uchar ACK_data_continuous[] =         {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};

// Laser Range Finder Controller
class lrf_controller
{
public:
    lrf_controller();

    ~lrf_controller();

    enum CAPTURE_MODE {
        ONCE,
        CONTINUOUS,
        STOP
    };

    bool open(QString comPortIn, int baudRateIn);

    bool isOpen() { return serial->isOpen(); }

    void pushToBuf();

    void requestData(int mode);

    void stopRetrieve();

    bool retrieveData(double *data, int mode);

    bool close();

    bool bufEnoughSet() {return buf.size() >= LENGTH_RAW_DATA;}

private:
    QSerialPort *serial;

    unsigned char data_raw[LENGTH_RAW_DATA - LENGTH_HEADER];

    QSerialPort::BaudRate baudRate;

    int mode;

    enum HEADER_TYPE{
        BAUDRATE,
        DATA
    };

    QByteArray buf;

    int count_while = 0;

    int break_count = 2;

    QByteArray dataSet;

    bool sendMsg(int mode);

    bool checkACK(QByteArray &data, const uchar *msg_ack);

    // check header =========
    bool fg_header;

    int data_size;

    int shiftContiMode;

    bool checkHeader(QByteArray &data, int header_type, int mode);
    // ======================

    ushort doCRC(const QByteArray &data);

};

#endif // LRF_CONTROLLER_H
