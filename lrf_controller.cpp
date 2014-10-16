#include "lrf_controller.h"


lrf_controller::lrf_controller()
{
    serial = new QSerialPort();
    baudRate = QSerialPort::Baud9600;

    // allocate size
    buf.reserve(MAX_BUF_SIZE);
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

bool lrf_controller::sendMsg(int mode)
{
    buf.clear();

    switch (mode) {
    default:
    case CAPTURE_MODE::ONCE:
        serial->write(request_data_once, 8);
        break;
    case CAPTURE_MODE::CONTINUOUS:
        serial->write(request_data_continuous, 8);
        break;

    case CAPTURE_MODE::STOP:
        serial->write(request_data_stop, 8);
        break;
    }

    count_while = 0;
//    checkACK();
    while(!serial->waitForBytesWritten(1)) {
        count_while++;
        if (count_while > break_count)
            return false;
    }

    this->mode = mode;

    return true;
}

void lrf_controller::requestData(int mode)
{
    qDebug()<<"sendMsg:"<<sendMsg(mode);
}

void lrf_controller::stopRetrieve()
{
    sendMsg(CAPTURE_MODE::STOP);
}

void lrf_controller::pushToBuf()
{
    if (buf.size() > MAX_BUF_SIZE)
        buf.clear();

    serial->waitForReadyRead(10);
    if (serial->bytesAvailable() > 0) {
        buf.append(serial->readAll());
#ifdef debug_info_lrf
        qDebug()<<"buf size: "<<buf.size();
#endif
#ifdef debug_info_lrf_data
        for (int i = 0; i < buf.size(); i++) {
            std::cout<<std::hex << (int)(buf[i] & 0xff)<<std::dec<<" ";
        }
        std::cout<<std::endl;
#endif
    }
#ifdef debug_info_lrf
    else
        qDebug()<<"serial: no data";
#endif

}

bool lrf_controller::retrieveData(double* data, int mode)
{
#ifdef debug_info_lrf
    qDebug()<<"retrieveData ====";
#endif

    shiftContiMode = 0;
    if (mode == CAPTURE_MODE::CONTINUOUS)
        shiftContiMode = 1;

//    while (buf.size() < LENGTH_RAW_DATA - shiftContiMode) {
//    }
//    count_while = 0;
    while (!checkHeader(buf, HEADER_TYPE::DATA, mode)) {
//        if (count_while > break_count) {
//#ifdef debug_info_lrf
//            qDebug()<<"header GG"<<buf.size();
//#endif
//            return false;
//        }
//        count_while++;
    }
#ifdef debug_info_lrf
    qDebug()<<"DONE check header";
#endif

    dataSet.clear();
    dataSet = buf.left(LENGTH_RAW_DATA - shiftContiMode);
    buf = buf.right(buf.size() - LENGTH_RAW_DATA + shiftContiMode);
//    count_while = 0;
//    while (dataSet.size() < LENGTH_RAW_DATA) {
//        int data_lack = LENGTH_RAW_DATA - dataSet.size();
//        dataSet.append(buf.left(data_lack));
//#ifdef debug_info_lrf
//        qDebug()<<dataSet.size();
//#endif
//        pushToBuf();
//        if (count_while > break_count)
//            return false;
//        count_while++;
//    }
//#ifdef debug_info_lrf
//    qDebug()<<"DONE add data";
//#endif

    for (int i = LENGTH_HEADER - shiftContiMode; i < LENGTH_RAW_DATA - shiftContiMode; i++) {
        unsigned char tp = dataSet[i];
        data_raw[i - LENGTH_HEADER + shiftContiMode] = tp;
    }
    int j = 0;
#ifdef debug_info_lrf_data
        qDebug()<<"proc data r";
#endif
    for (int i = 0; i < LENGTH_RAW_DATA - LENGTH_HEADER - 1; i+=2) {
        double r = (double)(data_raw[i + 1]) * 256 + (double)(data_raw[i]);
        data[j] = r;
#ifdef debug_info_lrf_data
        qDebug()<<r;
#endif
        j++;
    }

    return true;
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

bool lrf_controller::checkHeader(QByteArray &data, int header_type, int mode)
{
#ifdef debug_info_lrf
    qDebug()<<"check header "<<data.size();
#endif
    switch (header_type) {
    case HEADER_TYPE::BAUDRATE:
        break;
    case HEADER_TYPE::DATA:

#ifdef debug_info_lrf_data
        qDebug()<<"header";
        for (int i = 0 ; i < LENGTH_HEADER; i++)
            qDebug()<< i<<"\t"<< std::hex<<(int)header_data[i];
        qDebug()<<"data";
        for (int i = 0 ; i < data.size(); i++)
            qDebug()<< i<<"\t"<< std::hex<<(int)data.at(i);
#endif

        data_size = data.size();

        for (int i = 0; i < data_size - LENGTH_HEADER + shiftContiMode; i++) {
            fg_header = true;
            for (int j = shiftContiMode; j < LENGTH_HEADER; j++) {
                if (data.at(i + j - shiftContiMode) != header_data[j]) {
                    fg_header = false;
                    break;
                }
            }

            if (fg_header) {
#ifdef debug_info_lrf
                qDebug()<<"header before cut "<<data.size();
#endif
                data = data.mid(i);
#ifdef debug_info_lrf
                qDebug()<<"header after cut "<<data.size();
#endif
#ifdef debug_info_lrf_data
                qDebug()<<"data";
                for (int i = 0 ; i < data.size(); i++)
                    qDebug()<< std::hex<<(int)data.at(i);
#endif
                return fg_header;
            }
        }

        break;
    }

    return false;
}

bool lrf_controller::checkACK(QByteArray &data, const uchar *msg_ack)
{
    bool fg_ack_check = false;
    int data_size = data.size();

    for (int i = 0; i < data_size - LENGTH_ACK; i++) {
        fg_ack_check = true;
        for (int j = 0; j < LENGTH_ACK; j++) {
            if (data.at(i + j) != msg_ack[i + j]) {
                fg_ack_check = false;
                break;
            }
        }

        if (fg_ack_check) {
            data = data.right(data_size - i);
            return fg_ack_check;
        }
    }

    return fg_ack_check;
}
