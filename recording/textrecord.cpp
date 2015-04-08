#include "textrecord.h"

textRecord::textRecord()
{
    fg_record = false;

    fg_data_end = false;

    fg_file_established = false;
}

textRecord::~textRecord()
{

}

void textRecord::setPath(QString str, bool fg_str_is_file)
{
    if (fg_str_is_file) {
        file_name = str.section("/", -1, -1);
        file_path = str;
    }
    else {
        file_name = str.section("/", -1, -1) + ".txt";
        file_path = str + "/" + file_name;
    }
}

void textRecord::createText()
{
    file.setFileName(file_path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
//    if(!file.isOpen())
    out.setDevice(&file);

    fg_file_established = true;
//        return;
}

bool textRecord::record(std::string data)
{
    if (!fg_file_established)
        createText();

    if (fg_record) {
        if (out.status() == QTextStream::WriteFailed)
            return false;
        out << QString::fromStdString(data);
        out << "\n";
        return true;
    }

    return false;
}

void textRecord::stop()
{
    fg_file_established = false;
    fg_record = false;

    file.flush();
    file.reset();
    file.close();
    out.flush();
    out.reset();
    in.flush();
    in.reset();
}

bool textRecord::loadText(QString str, bool fg_str_is_file)
{
    setPath(str, fg_str_is_file);

    if (file_path.isEmpty())
        return false;
    file.setFileName(file_path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    if(!file.isOpen())
        return false;
    in.setDevice(&file);
//    qDebug()<<"1";
//    while(!in.atEnd()) {
//        QString text = file.readLine();
//        std::cout<<text.toStdString()<<std::endl;
//    }

    return true;
}
