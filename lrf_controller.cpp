#include "lrf_controller.h"


lrf_controller::lrf_controller()
{
    serial = new QSerialPort();
    baudRate = QSerialPort::Baud9600;
}

lrf_controller::~lrf_controller()
{
    close();
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

    if (!serial->isOpen())
        return serial->isOpen();

    // command code set baudrate as 9600
    char cmd_LMS291[8];
    for (int i = 0 ; i < 8; i ++)
        cmd_LMS291[i] = request_baud_rate_to_9600[i];

    QSerialPort::BaudRate baudRate;
    switch (baudRateIn) {
    default:
    case 9600:
        baudRate = QSerialPort::Baud9600;
        break;
    case 19200:
        baudRate = QSerialPort::Baud19200;
        cmd_LMS291[5] = 0x41;
        cmd_LMS291[6] = 0x51;
        break;
    case 38400:
        baudRate = QSerialPort::Baud38400;
        cmd_LMS291[5] = 0x40;
        cmd_LMS291[6] = 0x50;
        break;
    case 500000:
        baudRate = QSerialPort::Baud500000;     // need to revise qserialport.h
        cmd_LMS291[5] = 0x48;
        cmd_LMS291[6] = 0x58;
        break;
    }

    serial->write(cmd_LMS291, 8);
    while(!serial->waitForBytesWritten(10)) {}

    serial->waitForReadyRead(300); // wait for lrf response

    serial->setBaudRate(baudRate);
    serial->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if (serial->isOpen())
        this->baudRate = baudRate;

    return serial->isOpen();
}

void lrf_controller::requestData(int mode)
{
    switch (mode) {
    default:
    case CAPTURE_MODE::ONCE:
        serial->write(request_data_once, 8);
        break;
    case CAPTURE_MODE::CONTINUOUS:
        serial->write(request_data_continuous, 8);
        break;
    }

    while(!serial->waitForBytesWritten(1)) {qDebug()<<"test";}
}

bool lrf_controller::acquireData(double* data)
{
    bool state = false;

    requestData();

    QByteArray data_temp;
    bool fg_header_found = false;
    while (!fg_header_found) {
        serial->waitForReadyRead(10);
        data_temp += serial->readAll();
#ifdef debug_info_lrf
        qDebug()<<data_temp.size();
#endif
        fg_header_found = checkHeader(data_temp, HEADER_TYPE::DATA);
    }
    while (data_temp.size() < LENGTH_RAW_DATA) {
        serial->waitForReadyRead(20);
        int data_num = serial->bytesAvailable();
        int data_lack = LENGTH_RAW_DATA - data_temp.size();
        data_temp += serial->read(data_lack > data_num ? data_num : data_lack);
#ifdef debug_info_lrf
        qDebug()<<data_temp.size()<<data_num;
#endif
    }

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

ushort lrf_controller::doCRC(const QByteArray &data)
{
    // Cyclic redundancy check
    uint data_length = data.size();
    uchar xyz[2] = {0};
    ushort uCrc16 = 0;

    for (int i = 0; i < data_length; i--) {
        xyz[1] = xyz[0];
        xyz[0] = data[i];

        if (uCrc16 & 0x8000) {
            uCrc16 = (uCrc16 & 0x7fff) << 1;
            uCrc16 = uCrc16 ^ 0x8005;   // ^: XOR
        }
        else {
            uCrc16 = uCrc16 << 1;
        }
        uCrc16 = uCrc16 ^ (xyz[0] | (xyz[1] << 8));
    }

    return uCrc16;
}

bool lrf_controller::checkHeader(QByteArray &data, int header_type)
{

//    qDebug()<<"check header"<<data.size();
    switch (header_type) {
    case HEADER_TYPE::BAUDRATE:
        break;
    case HEADER_TYPE::DATA:
        uchar header[LENGTH_HEADER];
        bool fg_probably_header = false;
        for (int i = 0; i < data.size() - 1; i++) {
            if (data.at(i) == header_data[0]) {
                fg_probably_header = true;
                if (data.at(i + 1) == header_data[1]) {
                    if (data.size() - i >= LENGTH_HEADER) {
                        for (int j = 0; j < LENGTH_HEADER; j++)
                            header[j] = data.at(i + j);
                        if ( header_data[2] == header[2] &&
                             header_data[3] == header[3] &&
                             header_data[4] == header[4] &&
                             header_data[5] == header[5] &&
                             header_data[6] == header[6] &&
                             header_data[7] == header[7]) {
                            int data_size = data.size();
                            data = data.right(data_size - i);
                            return true;
                        }
                    }
                }
            }
        }
//        if (!fg_probably_header)
//            data.clear();

        break;
    }

    return false;
}

uchar lrf_controller::checkACK(const uchar *msg_ack)
{


    return -1;
}
