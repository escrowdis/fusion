#ifndef LRF_CONTROLLER_H
#define LRF_CONTROLLER_H

#include "debug_info.h"

#include <iostream>
#include <QString>
#include <QBuffer>
#include <QTime>

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock_lrf;

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

#define LRF_SHIFT_X 45  //**// unknown
#define LRF_MAP_WIDTH 722
#define LRF_MAP_HEIGHT 800

const char header_data[] =                 {0x06, 0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};
//! once mode: header_data pop up
//! continuous mode: ACK first and come up w/ header data w/o first uchar 0x06
//! SO, header in ONCE and CONTI is 7 and 8 respectively.

const char request_baud_rate_to_9600[] =    {0x02, 0x00, 0x02, 0x00, 0x20, 0x42, 0x52, 0x08};
const char request_data_once[] =            {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
const char request_data_continuous[] =      {0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08};
const char request_data_stop[] =            {0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x35, 0x08};

const uchar ACK_baudrate[] =                {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};
const uchar ACK_data_continuous[] =         {0x06, 0x02, 0x80, 0x03, 0x00, 0xA0, 0x00, 0x10, 0x16, 0x0A};

namespace LRF {

//!
//! \brief The CAPTURE_MODE enum
//!
enum CAPTURE_MODE {
    ONCE,           ///< capture data once
    CONTINUOUS,     ///< continuously retrieving data
    STOP            ///< stop retrieving data. Opposite to CONTINUOUS
};

enum HEADER_TYPE{
    BAUDRATE,
    DATA
};

}

//!
//! \brief The lrf_controller class is Laser Range Finder Controller
//!
class lrf_controller : public QObject
{
    Q_OBJECT

public:
    lrf_controller();

    ~lrf_controller();

    int time_proc, time_proc_buf;

    int scale_ratio = 2;

    double *lrf_data;               // (cm)

    cv::Mat display_lrf;

    cv::Mat display_lrf_BG;

    //!
    //! \brief open the serial port connection between PC and device
    //! \param comPortIn port number
    //! \param baudRateIn communication baud rate
    //! \return
    //!
    bool open(QString comPortIn, int baudRateIn);

    //!
    //! \brief isOpen check if the port is connected
    //! \return
    //!
    bool isOpen() { return serial->isOpen(); }

    //!
    //! \brief reset lrf data
    //!
    void reset();

    bool dataExec();

    bool guiUpdate();

    //!
    //! \brief pushToBuf put acquired info. from device into the \link buf.
    //!
    void pushToBuf();

    //!
    //! \brief requestData by sending request msg
    //! \param mode once and continuous\n
    //! once mode: header_data pop up\n
    //! continuous mode: ACK first and come up w/ header data w/o first uchar 0x06\n
    //! SO, header in ONCE and CONTI is 7 and 8 respectively.
    //!
    void requestData(int mode);

    void stopRetrieve();

    //!
    //! \brief retrieveData starts to retrieve the info. from device
    //! \param data the storage of device's info.
    //! \return
    //!
    bool retrieveData(double *data);

    bool close();

    //!
    //! \brief bufEnoughSet check if the info. in \link buf is more than a set for one process
    //! \return
    //!
    bool bufEnoughSet() {
        if (buf->size() >= dataset_size)
            return true;
        return false;
    }

    //!
    //! \brief bufNotFull the same as \link bufEnoughSet
    //! \return
    //!
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

    int mode;                           ///< current process mode

    // status
    QTime t_p, t_p_buf;                 ///< process time of all exec.
    QTime t;                            ///< control gui not to update too fast
    int time_gap;

    QByteArray *buf;                    ///< buffer

    // data size ============
    int dataset_size;                   ///< input data size

    int header_size;                    ///< input header size
    // ======================

    // buffer size params ===
    qint64 num_serial;                  ///< current info. in the buf

    qint64 num_lack;                    ///< lack how many number to construct one complete data

    qint64 num_input;                   ///< info. amount in the buf
    // ======================

    QByteArray dataSet;                 ///< cropped a complete data for use

    //!
    //! \brief sendMsg to call for the data
    //! \param mode
    //! \return
    //!
    bool sendMsg(int mode);

    //!
    //! \brief setMode to choose the mode of retrieving data
    //!
    void setMode();

    //!
    //! \brief checkACK [unused] to check the msg
    //! \param data
    //! \param msg_ack
    //! \return
    //!
    bool checkACK(QByteArray &data, const uchar *msg_ack);

    // check header =========
    bool fg_header;                 ///< check header

    //!
    //! \brief checkHeader to check the msg's header
    //! \param data
    //! \param header_type
    //! \return
    //!
    bool checkHeader(QByteArray &data, int header_type);
    // ======================

    ushort doCRC(const QByteArray &data);

signals:
    void updateGUI(double *data, cv::Mat *display_lrf);
};

#endif // LRF_CONTROLLER_H
