#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include "debug_info.h"

#include <iostream>
#include <QString>
#include <QTime>
#include <QBuffer>

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock;

// COM Port Communication
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <opencv2/opencv.hpp>

#define LENGTH_DATA 361
#define LENGTH_RAW_DATA_ONCE 733
#define LENGTH_HEADER_ONCE 8
#define LENGTH_RAW_DATA_CONTI 732
#define LENGTH_HEADER_CONTI 7
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

namespace LRF {

enum CAPTURE_MODE {
    ONCE,
    CONTINUOUS,
    STOP
};

enum HEADER_TYPE{
    BAUDRATE,
    DATA
};

}

// Laser Range Finder Controller
class lrf_controller : public QObject
{
    Q_OBJECT

public:
    lrf_controller();

    ~lrf_controller();


    bool open(QString comPortIn, int baudRateIn);

    bool isOpen() { return serial->isOpen(); }

    void pushToBuf();

    void requestData(int mode);

    void stopRetrieve();

    bool retrieveData(double *data);

    bool close();

    bool bufEnoughSet() {
        if (buf->size() >= dataset_size)
            return true;
        return false;
    }

    bool bufNotFull() {
#ifdef debug_info_lrf
        qDebug()<<"buf size: "<<buf->size();
#endif
        return buf->size() < MAX_BUF_SIZE;
    }

private:
    QSerialPort *serial;

    unsigned char data_raw[LENGTH_RAW_DATA_ONCE - LENGTH_HEADER_ONCE];

    QSerialPort::BaudRate baudRate;

    int mode;

    QByteArray *buf;

    // data size ============
    int dataset_size;

    int header_size;
    // ======================

    // buffer size params ===
    qint64 num_serial;

    qint64 num_lack;

    qint64 num_input;
    // ======================

    QByteArray dataSet;

    bool sendMsg(int mode);

    void setMode();

    bool checkACK(QByteArray &data, const uchar *msg_ack);

    // check header =========
    bool fg_header;

    bool checkHeader(QByteArray &data, int header_type);
    // ======================

    ushort doCRC(const QByteArray &data);
};

#endif // LRF_CONTROLLER_H
