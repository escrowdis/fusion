#ifndef TEXTRECORD_H
#define TEXTRECORD_H

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>
#include <QDebug>

#include <iostream>

class textRecord
{
public:
    textRecord();

    ~textRecord();

    bool loadText(QString str, bool fg_str_is_file);

    void setPath(QString str, bool fg_str_is_file);

    bool record(std::string data);

    void stop();

    bool fg_record;

    bool fg_data_end;

    QTextStream out;

    QTextStream in;

    QFile file;

private:
    void defaultBasicInfo();

    void createText();

    QString file_path;

    QString file_name;

    bool fg_file_established;

    void createVideo();
};

#endif // TEXTRECORD_H
