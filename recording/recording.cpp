#include "recording.h"

recording::recording()
{
    fg_parent_existed = false;

    fg_folder_existed = false;

    vr = new videoRecord();

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

void recording::setRecordType(RECORD_TYPE type)
{
    record_type = type;
}

//void recording::setBasicInfo(cv::VideoCapture *cap)
//{
//    vr->getBasicInfo(cap);
//}

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

bool recording::loadData()
{
    QString data_name, data_dir;
    switch (record_type) {
    case RECORD_TYPE::VIDEO:
        data_name = QFileDialog::getOpenFileName(0, "data file", 0, QFileDialog::tr("Video file (*.avi)"));
        if (data_name.isEmpty())
            return false;
        vr->loadVideo(data_name, true);
        break;
    case RECORD_TYPE::TXT:
        data_name = QFileDialog::getOpenFileName(0, "data file", 0, QFileDialog::tr("Text file (*.txt)"));
        if (data_name.isEmpty())
            return false;
        tr->loadText(data_name, true);
        break;
    case RECORD_TYPE::ALL:
        data_dir = QFileDialog::getExistingDirectory(0, "data folder", 0, QFileDialog::ShowDirsOnly);
        if (data_dir.isEmpty())
            return false;
        vr->loadVideo(data_dir, false);
        tr->loadText(data_dir, false);
        break;
    }

    return true;
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
