#include "textrecord.h"

textRecord::textRecord()
{
    fg_record = false;

    fg_loaded = false;

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
    file.open(file_path.toStdString(), std::fstream::out | std::fstream::app);

    fg_file_established = true;
//        return;
}

bool textRecord::record(std::string data)
{
    if (!fg_file_established)
        createText();

    if (fg_record) {
        if (!file) {
            std::cout<<"Failed to open txt file for writing."<<std::endl;
            return false;
        }
        file << data;
        file << "\n";
        return true;
    }

    return false;
}

void textRecord::stop()
{
    fg_file_established = false;
    fg_record = false;
    fg_loaded = false;

    file.close();
    file.flush();
}

bool textRecord::loadText(QString str, bool fg_str_is_file)
{
    setPath(str, fg_str_is_file);

    if (file_path.isEmpty()) {
        fg_loaded = false;
        return fg_loaded;
    }
    if (file.is_open())
        file.close();
    file.open(file_path.toStdString());
    if(!file) {
        fg_loaded = false;
        return fg_loaded;
    }

    current_frame_count = 0;

    fg_loaded = true;
    return fg_loaded;
}
