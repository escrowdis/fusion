#include "lrf_controller.h"

lrf_controller::lrf_controller()
{
    serial = new QSerialPort();
}

bool lrf_controller::open(QString comPortIn, int baudRateIn)
{
    QSerialPort::BaudRate baudRate;
    switch (baudRateIn) {
    case 1200:
        baudRate = QSerialPort::Baud1200;
        break;
    case 2400:
        baudRate = QSerialPort::Baud2400;
        break;
    case 4800:
        baudRate = QSerialPort::Baud4800;
        break;
    case 9600:
        baudRate = QSerialPort::Baud9600;
        break;
    case 19200:
        baudRate = QSerialPort::Baud19200;
        break;
    case 38400:
        baudRate = QSerialPort::Baud38400;
        break;
    case 57600:
        baudRate = QSerialPort::Baud57600;
        break;
    case 115200:
        baudRate = QSerialPort::Baud115200;
        break;

    default:
        baudRate = QSerialPort::Baud38400;
    }

    serial->close();
    serial->setPortName(comPortIn);
    serial->setBaudRate(baudRate);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setDataBits(QSerialPort::Data8);
    serial->setStopBits(QSerialPort::OneStop);
    serial->open(QIODevice::ReadWrite);

    char LMS291_38400_cmd[8] = {0x02, 0x00, 0x02, 0x00, 0x20, 0x40, 0x50, 0x08};
    serial->write(LMS291_38400_cmd, 8);

    serial->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    return serial->isOpen();
}

void lrf_controller::request()
{
    char alpha[8] = {0x02, 0x00, 0x02, 0x00, 0x30, 0x01, 0x31, 0x18};
    serial->write(alpha, 8);
}

void lrf_controller::acquireData(double* data)
{
    request();

    QThread::msleep(50); // wait lrf


}
