#include "calibrationform.h"
#include "ui_calibrationform.h"

calibrationForm::calibrationForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::calibrationForm)
{
    ui->setupUi(this);

    cc = new camera_calibration;

    reset();
}

void calibrationForm::reset()
{
    CCD = 'L';

    ui->label_recordedL->setText(QString::number(0));
    ui->label_recordedR->setText(QString::number(0));

    ui->label_status->setText("Record left image");

    // focus on ui w/o editing lineEdit or something
    setFocus();
}

calibrationForm::~calibrationForm()
{
    cv::destroyAllWindows();
    delete cc;
    delete ui;
}

void calibrationForm::on_pushButton_3_clicked()
{
    close();
}

void calibrationForm::on_pushButton_corner_intrinsic_clicked()
{
    // load images
    image_save_folder = QFileDialog::getExistingDirectory();
    if (image_save_folder.isEmpty())
        return;
    image_save_folder += "/";

    std::vector <std::string> files[2];
    loadFiles(image_save_folder, files);

    cv::Size size_pattern = cv::Size(ui->spinBox_corners_cols->value(), ui->spinBox_corners_rows->value());
    cv::Size2f size_grid = cv::Size2f(ui->lineEdit_grid_w->text().toFloat(), ui->lineEdit_grid_h->text().toFloat());

    cc->CameraCalibration(true, size_pattern, size_grid, files[0]);
    cc->SaveIntrinsic(image_save_folder.toStdString(), std::string("camL_init.yml"));
    cc->DisplayUndistortedImg(false);
    cc->CameraCalibration(true, size_pattern, size_grid, files[1]);
    cc->SaveIntrinsic(image_save_folder.toStdString(), std::string("camR_init.yml"));
    cc->DisplayUndistortedImg(false);
}

void calibrationForm::on_pushButton_calibration_clicked()
{
    ui->progressBar->setValue(0);
    ui->label_status->setText("Start calibration");

    // stereo match
}

void calibrationForm::loadFiles(QString folder, std::vector <std::string> files[])
{
    QStringList filter_L("*_L.jpg");
    QStringList filter_R("*_R.jpg");
    images_L = QDir(folder).entryList(filter_L);
    images_R = QDir(folder).entryList(filter_R);
    for (int i = 0; i < images_L.size(); i++)
        files[0].push_back(folder.toStdString() + images_L[i].toStdString());
    for (int i = 0; i < images_R.size(); i++)
        files[1].push_back(folder.toStdString() + images_R[i].toStdString());
}

void calibrationForm::nextCam()
{
    if (CCD == 'L') {
        CCD = 'R';
        ui->label_status->setText("Record right image");
    }
}

void calibrationForm::prevCam()
{
    if (CCD == 'R') {
        CCD = 'L';
        ui->label_status->setText("Record left image");
    }
}

void calibrationForm::mouseReleaseEvent(QMouseEvent *event)
{
#ifdef debug_info_cc
    qDebug()<<event->button()<<event->buttons();
#endif
    switch (event->button()) {
    case Qt::MouseButton::LeftButton:   // left button
#ifdef debug_info_cc
        qDebug()<<"left";
#endif
        emit requestImage(CCD);
        break;
    case Qt::MouseButton::RightButton:   // right button
#ifdef debug_info_cc
        qDebug()<<"right";
#endif
        nextCam();
        break;
    case Qt::MouseButton::MidButton:    // mid button
        prevCam();
        break;
    }
}

void calibrationForm::keyReleaseEvent(QKeyEvent *event)
{
#ifdef debug_info_cc
    qDebug()<<event->key();
#endif
    switch (event->key()) {
    case 83:    // s or S
#ifdef debug_info_cc
        qDebug()<<"'s' char typed";
#endif
        emit requestImage(CCD);
        break;
    case 78:    // n
        nextCam();
        break;
    case 66:    // b
        prevCam();
        break;
    case 81:    // q
        cv::destroyAllWindows();
        break;
    case Qt::Key_Escape:
        cv::destroyAllWindows();
        break;
    }
}

void calibrationForm::saveImage(cv::Mat *img)
{
#ifdef debug_info_cc
    qDebug()<<"save Image";
#endif

    if (!image_save_path.exists(image_save_folder))
        image_save_path.mkdir(image_save_folder);

    // image file name
    t_now_string = t_now.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
    if (CCD == 'L') {
        // save image
        file_save_L = image_save_folder + "/" + t_now_string + "_L.jpg";
        cv::cvtColor(*img, img_s_L, cv::COLOR_BGR2RGB);
        cv::imwrite(file_save_L.toStdString(), img_s_L);
        ui->label_recordedL->setText(QString::number(ui->label_recordedL->text().toInt() + 1));

        // display
        cv::namedWindow(image_name_L, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(image_name_L, 100, 600);
        cv::imshow(image_name_L, img_s_L);

        // set focus on widget
        activateWindow();
    }
    else if (CCD == 'R') {
        // save image
        file_save_R = image_save_folder + "/" + t_now_string + "_R.jpg";
        cv::cvtColor(*img, img_s_R, cv::COLOR_BGR2RGB);
        cv::imwrite(file_save_R.toStdString(), img_s_R);
        ui->label_recordedR->setText(QString::number(ui->label_recordedR->text().toInt() + 1));

        // display
        cv::namedWindow(image_name_R, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(image_name_R, 500, 600);
        cv::imshow(image_name_R, img_s_R);

        // set focus on widget
        activateWindow();
    }
}

void calibrationForm::saveImages(cv::Mat *img_L, cv::Mat *img_R)
{
    if (!image_save_path.exists(image_save_folder + "_both"))
        image_save_path.mkdir(image_save_folder + "_both");

    t_now_string = t_now.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
    file_save_L = image_save_folder + "/" + t_now_string + "_L.jpg";
    file_save_R = image_save_folder + "/" + t_now_string + "_R.jpg";
    cv::cvtColor(*img_L, img_s_L, cv::COLOR_BGR2RGB);
    cv::cvtColor(*img_R, img_s_R, cv::COLOR_BGR2RGB);

    cv::imwrite(file_save_L.toStdString(), img_s_L);
    cv::imwrite(file_save_R.toStdString(), img_s_R);

    cv::namedWindow(image_name_L, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(image_name_L, 100, 600);
    cv::namedWindow(image_name_R, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(image_name_R, 500, 600);

    cv::imshow(image_name_L, img_s_L);
    cv::imshow(image_name_R, img_s_R);
}

void calibrationForm::on_checkBox_SaveBoth_clicked(bool checked)
{
    if (checked) {
        CCD_temp = CCD;
        CCD = 'B';
        ui->label_status->setText("Record both images");
    }
    else {
        CCD = CCD_temp;
        if (CCD == 'L')
            ui->label_status->setText("Record left image");
        else if (CCD == 'R')
            ui->label_status->setText("Record right image");
    }
}

void calibrationForm::getBasicInfo(int focal_length, double base_line)
{
#ifdef debug_info_cc
    qDebug()<<"f: "<<focal_length<<"B:"<<base_line;
#endif

    this->focal_length = focal_length;
    this->base_line = base_line;

    // set folder named by time
    image_save_folder = "calibrationUsedImgs";
    image_save_path = QDir::currentPath();
    QString current_folder = image_save_path.path().section("/", -1, -1);
    if (current_folder == "release" || current_folder == "debug")
        image_save_path.cdUp();
    else if (current_folder != "Fusion")
        return;
    if (!image_save_path.exists(image_save_folder))
        image_save_path.mkdir(image_save_folder);
    image_save_path.cd(image_save_folder);
    image_save_folder = image_save_path.path() + "/" +
            t_now.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss") +
            "_F" + QString::number(focal_length) +
            "_B" + QString::number(base_line);

#ifdef debug_info_cc
    qDebug()<<image_save_folder;
#endif
}
