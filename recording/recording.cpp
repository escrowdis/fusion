#include "recording.h"

recording::recording(int img_h, int img_w)
{
    fg_parent_existed = false;

    fg_folder_existed = false;

    vr = new videoRecord(img_h, img_w);

    tr = new textRecord();
}

recording::~recording()
{
    delete vr;
    delete tr;
}

void recording::setParentFolder(QString folder)
{
    // set folder named by time
    cwd = project_path;
    parent_folder = folder;
    if (!cwd.exists(parent_folder))
        cwd.mkdir(parent_folder);
    cwd.cd(parent_folder);
    parent_dir = cwd;

    fg_parent_existed = true;
}

void recording::createFolder()
{
    cwd = parent_dir;
    save_folder = t_save = t_now.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
    if (!cwd.exists(save_folder))
        cwd.mkdir(save_folder);
    cwd.cd(save_folder);

    vr->setPath(cwd.path(), false);
    tr->setPath(cwd.path(), false);

    fg_folder_existed = true;
}

void recording::setRecordType(int record_type)
{
    this->record_type = record_type;
}

void recording::setBasicInfo(cv::VideoCapture &cap)
{
    vr->getBasicInfo(cap);
}

bool recording::recordData(cv::Mat img)
{
    if (!fg_parent_existed)
        return false;

    if (!fg_folder_existed)
        createFolder();

    return vr->record(img);
}

bool recording::recordData(std::string data)
{
    if (!fg_parent_existed)
        return false;

    if (!fg_folder_existed)
        createFolder();

    return tr->record(data);
}

QString recording::loadData()
{
    QString data_name, data_dir;
    switch (record_type) {
    case RECORD_TYPE::VIDEO:
        data_name = QFileDialog::getOpenFileName(0, "data file", 0, QFileDialog::tr("Video file (*.avi)"));
        if (data_name.isEmpty())
            return "";
        vr->loadVideo(data_name, true);

        return data_name;
        break;
    case RECORD_TYPE::TXT:
        data_name = QFileDialog::getOpenFileName(0, "data file", 0, QFileDialog::tr("Text file (*.txt)"));
        if (data_name.isEmpty())
            return "";
        tr->loadText(data_name, true);

        return data_name;
        break;
    case RECORD_TYPE::ALL:
        data_dir = QFileDialog::getExistingDirectory(0, "data folder", 0, QFileDialog::ShowDirsOnly);
        if (data_dir.isEmpty())
            return "";
        vr->loadVideo(data_dir, false);
        tr->loadText(data_dir, false);

        return data_dir;
        break;
    }
}

void recording::start(RECORD_TYPE type)
{
    setRecordType(type);
    switch (record_type) {
    case RECORD_TYPE::TXT:
        tr->fg_record = true;
        break;
    case RECORD_TYPE::VIDEO:
        vr->fg_record = true;
        break;
    case RECORD_TYPE::ALL:
        tr->fg_record = true;
        vr->fg_record = true;
        break;
    }

}

void recording::stop()
{
    fg_folder_existed = false;

    if (vr->fg_record)
        vr->stop();
    if (tr->fg_record)
        tr->stop();
}
