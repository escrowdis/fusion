#include "lrf_controller.h"

lrf_controller::lrf_controller()
{
    serial = new QSerialPort();
    baudRate = QSerialPort::Baud9600;
}

bool lrf_controller::open(QString comPortIn, int baudRateIn)
{
    serial->close();
    serial->setPortName(comPortIn);
    serial->setBaudRate(this->baudRate);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setDataBits(QSerialPort::Data8);
    serial->setStopBits(QSerialPort::OneStop);
    serial->open(QIODevice::ReadWrite);

    char LMS291_cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x42, 0x52, 0x08};

    QSerialPort::BaudRate baudRate;
    switch (baudRateIn) {
    case 9600:
        baudRate = QSerialPort::Baud9600;
        break;
    case 19200:
        baudRate = QSerialPort::Baud19200;
        LMS291_cmd[5] = 0x41;
        LMS291_cmd[6] = 0x51;
        break;
    case 38400:
        baudRate = QSerialPort::Baud38400;
        LMS291_cmd[5] = 0x40;
        LMS291_cmd[6] = 0x50;
        break;
    case 57600:
        baudRate = QSerialPort::Baud57600;
        break;
    case 115200:
        baudRate = QSerialPort::Baud115200;
        break;

    default:
        baudRate = QSerialPort::Baud9600;
    }

    serial->write(LMS291_cmd, 8);
    serial->waitForReadyRead(300); // wait for lrf response

    serial->setBaudRate(baudRate);
    serial->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if (serial->isOpen())
        this->baudRate = baudRate;

    return serial->isOpen();
}

void lrf_controller::request()
{
    char alpha[8] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
    serial->write(alpha, 8);
}

bool lrf_controller::acquireData(double* data)
{
    bool state = false;

    request();

    serial->waitForReadyRead(500); // wait for lrf response

    int data_num = serial->bytesAvailable();

    if (data_num > 0 && data_num < 1024) {
        QByteArray data_temp = serial->read(data_num);
#ifdef debug_info_lrf
        qDebug()<<data_num;
#endif
        if (data_temp.size() == LENGTH_RAW_DATA) {
            for (int i = LENGTH_HEADER; i < LENGTH_RAW_DATA; i++) {
                unsigned char tp = data_temp[i];
                data_raw[i - LENGTH_HEADER] = tp;
            }
            int j = 0;
            for (int i = 0; i < LENGTH_RAW_DATA - LENGTH_HEADER - 1; i+=2) {
                double r = (double)(data_raw[i + 1]) * 256 + (double)(data_raw[i]);
                data[j] = r;
#ifdef debug_info_lrf
                qDebug()<<r;
#endif
                j++;
            }

            state = true;
        }
    }
#ifdef debug_info_lrf
    qDebug()<<state;
#endif
    return state;
}

bool lrf_controller::close()
{
    serial->close();
    return serial->isOpen();
}
