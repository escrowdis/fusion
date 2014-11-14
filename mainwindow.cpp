#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    form_calib(0), form_smp(0)
{
    ui->setupUi(this);

    fg_running = false;

    qRegisterMetaType<cv::Mat>("cv::Mat");

    // Laser range finder ======================

    // Initialization
    lrf = new lrf_controller();

    fg_acquiring = false;
    fg_buffering = false;

    QObject::connect(this, SIGNAL(updateGUI()), this, SLOT(lrfDisplay()));

    // COM port
    for (int i = 1; i <= 20; i++)
        ui->comboBox_lrf_com->addItem("COM" + QString::number(i));

    // Baud rate
    QStringList list_baudrate;
    list_baudrate << "9600" << "19200" << "38400" << "500000";
    ui->comboBox_lrf_baudRate->addItems(list_baudrate);

    // default settings
    lrfClearData();

    // display
    display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);

    // Laser range finder ====================== End

    // Stereo vision ===========================

    // Initialization
    sv = new stereo_vision();

    fg_capturing = false;

    QObject::connect(sv, SIGNAL(updateGUI(cv::Mat *, cv::Mat *, cv::Mat *)), this, SLOT(svDisplay(cv::Mat *, cv::Mat *, cv::Mat *)));

    // COM port
    for (int i = 0; i < 5; i++) {
        ui->comboBox_cam_device_index_L->addItem(QString::number(i));
        ui->comboBox_cam_device_index_R->addItem(QString::number(i));
    }

    // focal length
    ui->comboBox_camera_focal_length->addItem("16");
    ui->comboBox_camera_focal_length->addItem("12");
    ui->comboBox_camera_focal_length->addItem("4");

    // Stereo vision =========================== End

    // Thread control ==========================
    f_sv.setPaused(true);
    f_lrf.setPaused(true);
    f_lrf_buf.setPaused(true);
    sync.addFuture(f_sv);
    sync.addFuture(f_lrf);
    sync.addFuture(f_lrf_buf);
    // ========================================= End

    // camera calibration ======================
    // ========================================= End

    // default setting =========================
    if (!projectFolder()) {
        reportError("path", "Error!", "Path cannot be found.");
        return;
    }

    paramRead();
    // ========================================= End
}

MainWindow::~MainWindow()
{
#ifdef debug_info_main
    qDebug()<<"destructor";
#endif
    paramWrite();

    cv::destroyAllWindows();
    lrf->close();
    delete lrf;
    delete sv;
    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
    delete ui;
}

void MainWindow::reportError(QString part, QString level, QString content)
{
    ui->system_log->append("<FONT COLOR = red>[" + part + "] " + level + "  " + content + "</FONT>");
    QMessageBox::information(0, level, content);
}

void MainWindow::report(QString content)
{
    ui->system_log->append(content);
}

bool MainWindow::projectFolder()
{
    project_path = QDir::currentPath();
    QString current_folder = project_path.currentPath().section("/", -1, -1);
    if (current_folder == "release" || current_folder == "debug")  {
        project_path.cdUp();
    }
    if (project_path.path().section("/", -1, -1) == "Fusion") {
        return true;
    }

    return false;
}

void MainWindow::paramRead()
{
    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::READ);

    cv::FileNode n;
    n = fs["stereoVision"];
    ui->comboBox_cam_device_index_L->setCurrentIndex((int) n["port_L"]);
    ui->comboBox_cam_device_index_R->setCurrentIndex((int) n["port_R"]);
    ui->comboBox_camera_focal_length->setCurrentIndex((int) n["cam_focal_length"]);
    sv->cam_focal_length = ui->comboBox_camera_focal_length->currentText().toInt();
    sv->base_line = (double) n["base_line"];
    sv->focal_length = (double) n["focal_length"];
    ui->lineEdit_base_line->setText(QString::number(sv->base_line));
    ui->label_sv_focal_length->setText(QString::number(sv->focal_length));

    n = fs["laserRangeFinder"];
    ui->comboBox_lrf_com->setCurrentIndex((int) n["port"]);
    ui->comboBox_lrf_baudRate->setCurrentIndex((int) n["baudRate"]);
    ui->spinBox_lrf_scale->setValue((double) n["mapScale"]);
    std::string res = (std::string) n["resolution"];
    if (res == "cm")
        ui->radioButton_lrf_res_cm->setChecked(true);
    else if (res == "mm")
        ui->radioButton_lrf_res_mm->setChecked(true);

    fs.release();
}

void MainWindow::paramWrite()
{
    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::WRITE);

    fs << "stereoVision" << "{";
    fs << "port_L" << ui->comboBox_cam_device_index_L->currentIndex();
    fs << "port_R" << ui->comboBox_cam_device_index_R->currentIndex();
    fs << "cam_focal_length" << ui->comboBox_camera_focal_length->currentIndex();
    fs << "focal_length" << sv->focal_length;
    fs << "base_line" << ui->lineEdit_base_line->text().toDouble();
    fs << "}";

    fs << "laserRangeFinder" << "{";
    fs << "port" << ui->comboBox_lrf_com->currentIndex();
    fs << "baudRate" << ui->comboBox_lrf_baudRate->currentIndex();
    fs << "mapScale" << ui->spinBox_lrf_scale->value();
    if (ui->radioButton_lrf_res_cm->isChecked())
        fs << "resolution" << "cm";
    else if (ui->radioButton_lrf_res_mm->isChecked())
        fs << "resolution" << "mm";
    fs << "}";

    fs.release();
}

void MainWindow::on_pushButton_lrf_open_clicked()
{
    if (!lrf->open(ui->comboBox_lrf_com->currentText(), ui->comboBox_lrf_baudRate->currentText().toInt())) {
        reportError("lrf", "Error!", "Port cannot be open or under using.");
        return;
    }
    else {
        report("Port's opened.");
    }
}

void MainWindow::lrfClearData()
{
    // reset data
    for (int i = 0; i < LENGTH_DATA; i++)
        lrf_data[i] = -1.0; //**// lrf range?
}

bool MainWindow::lrfReadData()
{
//    lrfClearData();
    if (lrf->retrieveData(lrf_data)) {
        emit updateGUI();
        return true;
    }
    else
        return false;
}

void MainWindow::lrfDisplay()
{
    // display
    cv::Mat display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);
//    display_lrf.setTo(0);
    double angle = 0.0;

//    for (int i = 0 ; i < LENGTH_DATA; ++i)
//        if (lrf_data[i] == -1)
//            return;

    lock.lockForRead();
    for (int i = 0; i < LENGTH_DATA; ++i) {
        double r = lrf_data[i];
        double x = r * cos(angle * CV_PI / 180.0);
        double y = r * sin(angle * CV_PI / 180.0);

        cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, 800 - (y / ui->spinBox_lrf_scale->value() + 100)), 1, cv::Scalar(0, 0, 255), -1);
        if (LENGTH_DATA / 2 == i)
            cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, 800 - (y / ui->spinBox_lrf_scale->value() + 100)), 5, cv::Scalar(0, 255, 0), -1);

        angle += RESOLUTION;
    }

//    ui->label_lrf_data->setPixmap(QPixmap::fromImage(QImage::QImage(display_lrf.data, display_lrf.cols, display_lrf.rows, 3 * display_lrf.cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    cv::imshow("image", display_lrf);
    cv::waitKey(10);
    lock.unlock();
}

void MainWindow::on_pushButton_lrf_display_clicked()
{

}

void MainWindow::on_pushButton_lrf_stop_clicked()
{

}

void MainWindow::camOpen()
{
    int L = ui->comboBox_cam_device_index_L->currentIndex();
    int R = ui->comboBox_cam_device_index_R->currentIndex();

    if (!sv->open(L, R)) {
        reportError("sv", "Error!", "Camera can NOT be opened.");
        sv->close();
        return;
    }
    else {
        report("Port's opened.");
    }
}

void MainWindow::svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp)
{
#ifdef debug_info_sv
    qDebug()<<"disp"<<&img_L;
#endif
    lock.lockForRead();
    ui->label_cam_img_L->setPixmap(QPixmap::fromImage(QImage::QImage(img_L->data, img_L->cols, img_L->rows, 3 * img_L->cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_cam_img_R->setPixmap(QPixmap::fromImage(QImage::QImage(img_R->data, img_R->cols, img_R->rows, 3 * img_R->cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp->data, disp->cols, disp->rows, disp->cols, QImage::Format_Indexed8)).scaled(IMG_DIS_W, IMG_DIS_H));
    lock.unlock();
#ifdef debug_info_sv
    qDebug()<<"disp - End"<<&img_L;
#endif
}

void MainWindow::on_pushButton_cam_open_clicked()
{
    camOpen();
    on_pushButton_cam_step_clicked();
}

void MainWindow::on_pushButton_cam_step_clicked()
{
    if (!sv->isOpened()) {
        reportError("sv", "Error!", "Cameras haven't opened.");
        return;
    }

    sv->stereoVision();
}

void MainWindow::on_pushButton_cam_capture_clicked()
{
    if (!sv->isOpened()) {
        reportError("sv", "Error!", "Cameras haven't opened.");
        return;
    }

    fg_capturing = true;
    f_sv.setPaused(false);
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_cam_stop_clicked()
{
    fg_capturing = false;
    f_sv.setPaused(true);
}

void MainWindow::closeEvent(QCloseEvent *)
{
    // releas form since it's allocated
    if (form_calib != 0)
        delete form_calib;
    if (form_smp != 0)
        delete form_smp;    //**// memory location changed itself
    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
}

void MainWindow::threadbuffering()
{
    // unimplement
    while (fg_buffering) {
//        f_lrf_buf = QtConcurrent::run(lrf, &lrf_controller::pushToBuf);
//        fw_lrf_buf.setFuture(f_lrf_buf);
//        fw_lrf_buf.waitForFinished();
        qApp->processEvents();
    }
}

void MainWindow::threadProcessing()
{
    fg_running = true;
    t_proc.restart();
    while (fg_running) {
        // sv
        if (fg_capturing && !f_sv.isRunning()) {
//            sv->stereoVision();
            f_sv = QtConcurrent::run(sv, &stereo_vision::stereoVision);
        }
        ui->label_sv_proc->setText(QString::number(t_proc.restart()));

        // lrf
        if (fg_acquiring && !f_lrf.isRunning()) {
//            lrfReadData();
            f_lrf = QtConcurrent::run(this, &MainWindow::lrfReadData);
        }
        ui->label_lrf_proc->setText(QString::number(t_proc.restart()));

        // lrf buffer
        if (fg_buffering && lrf->bufNotFull() && !f_lrf_buf.isRunning()) {
//            lrf->pushToBuf();
            f_lrf_buf = QtConcurrent::run(lrf, &lrf_controller::pushToBuf);
        }
        ui->label_lrf_buf_proc->setText(QString::number(t_proc.restart()));

        sync.waitForFinished();

        qApp->processEvents();
    }

    sync.setCancelOnWait(true);
    while (!sync.cancelOnWait()) {qDebug()<<"wait";}
}

void MainWindow::on_pushButton_4_clicked()
{

}

void MainWindow::on_checkBox_do_calibration_clicked(bool checked)
{
    if (checked) {
        // load camera calibration files
        if (!sv->loadRemapFile(ui->comboBox_camera_focal_length->currentText().toInt(), ui->lineEdit_base_line->text().toDouble())) {
            reportError("cc", "Error!", "Calibration files can NOT be imported.");
            ui->checkBox_do_calibration->setChecked(false);
        }
        sv->fg_calib = true;
    }
    else
        sv->fg_calib = false;
}

void MainWindow::on_checkBox_do_depth_clicked(bool checked)
{
    if (checked) {
        sv->fg_stereoMatch = true;
    }
    else
        sv->fg_stereoMatch = false;
}

void MainWindow::on_pushButton_camera_calibration_clicked()
{
    if (form_calib == 0) {
        form_calib = new calibrationForm();
        form_calib->move(1500, 500);

        // send focal length & base line to form
        QObject::connect(this, SIGNAL(sendBasicInfo(int, double)), form_calib, SLOT(getBasicInfo(int, double)));
        QObject::connect(form_calib, SIGNAL(requestImage(char)), this, SLOT(requestImage(char)));
        QObject::connect(this, SIGNAL(sendImage(cv::Mat *)), form_calib, SLOT(saveImage(cv::Mat *)));
        QObject::connect(this, SIGNAL(sendImages(cv::Mat *, cv::Mat *)), form_calib, SLOT(saveImages(cv::Mat *, cv::Mat *)));
    }
    else
        form_calib->reset();
    emit sendBasicInfo(ui->comboBox_camera_focal_length->currentText().toInt(), ui->lineEdit_base_line->text().toDouble());

    form_calib->show();
}

void MainWindow::requestImage(char CCD)
{
    if (!sv->isOpened()) {
        reportError("sv", "Error!", "Cameras haven't opened.");
        return;
    }

    switch (CCD) {
    case 'L':
#ifdef debug_info_cc
        qDebug()<<"send left image";
#endif
        emit sendImage(&sv->img_r_L);
        break;
    case 'R':
#ifdef debug_info_cc
        qDebug()<<"send right image";
#endif
        emit sendImage(&sv->img_r_R);
        break;
    case 'B':
#ifdef debug_info_cc
        qDebug()<<"send both images";
#endif
        emit sendImages(&sv->img_r_L, &sv->img_r_R);
        break;
    }

}

void MainWindow::on_radioButton_BM_clicked()
{
    report("Change to BM mathod.");
    sv->matchParamInitialize(SV::STEREO_MATCH::BM);
}

void MainWindow::on_radioButton_SGBM_clicked()
{
    report("Change to SGBM mathod.");
    sv->matchParamInitialize(SV::STEREO_MATCH::SGBM);
}

void MainWindow::on_pushButton_stereo_match_param_clicked()
{
    if (form_smp == 0) {
        form_smp = new stereoMatchParamForm();
        form_smp->move(1500, 100);

//        QObject::connect(form_smp, SIGNAL(send_pre_filter_size(int)), sv, SLOT(change_pre_filter_size(int)));
//        QObject::connect(form_smp, SIGNAL(send_pre_filter_cap(int)), sv, SLOT(change_pre_filter_cap(int)));
//        QObject::connect(form_smp, SIGNAL(send_sad_window_size(int)), sv, SLOT(change_sad_window_size(int)));
//        QObject::connect(form_smp, SIGNAL(send_min_disp(int)), sv, SLOT(change_min_disp(int)));
//        QObject::connect(form_smp, SIGNAL(send_num_of_disp(int)), sv, SLOT(change_num_of_disp(int)));
//        QObject::connect(form_smp, SIGNAL(send_texture_thresh(int)), sv, SLOT(change_texture_thresh(int)));
//        QObject::connect(form_smp, SIGNAL(send_uniqueness_ratio(int)), sv, SLOT(change_uniqueness_ratio(int)));
//        QObject::connect(form_smp, SIGNAL(send_speckle_window_size(int)), sv, SLOT(change_speckle_window_size(int)));
//        QObject::connect(form_smp, SIGNAL(send_speckle_range(int)), sv, SLOT(change_speckle_range(int)));
    }

    // send cuurent stereo matching params to ui //**// undone
//    emit sendCurrentParams();

    form_smp->show();
}

void MainWindow::on_comboBox_camera_focal_length_currentIndexChanged(int index)
{
    if (fg_capturing) {
        if (ui->checkBox_do_calibration->isChecked())
            on_checkBox_do_calibration_clicked(true);
        report("focal length has been changed.");
    }
}

void MainWindow::on_lineEdit_base_line_returnPressed()
{
    if (fg_capturing) {
        on_checkBox_do_calibration_clicked(true);
        report("base line has been changed.");
    }
}

void MainWindow::on_pushButton_8_clicked()
{
    lrf->requestData(LRF::CAPTURE_MODE::ONCE);
    while (!lrf->bufEnoughSet()) {
        lrf->pushToBuf();
    }
    lrfReadData();
}

void MainWindow::on_pushButton_5_clicked()
{
    lrf->requestData(LRF::CAPTURE_MODE::CONTINUOUS);

    // push data to buffer //**// shouldn't be here
    fg_buffering = true;
    f_lrf_buf.setPaused(false);
//        lrf->bufRunning();
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_6_clicked()
{
    fg_acquiring = true;
    f_lrf.setPaused(false);
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_7_clicked()
{
    fg_buffering = false;
    fg_acquiring = false;
    f_lrf.setPaused(true);
    f_lrf_buf.setPaused(true);

    while (!(f_lrf.isPaused() && f_lrf_buf.isPaused())) {}
    lrf->stopRetrieve();

    qDebug()<<"stop done";
}
