#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    fg_form_created = false;

    fg_running = false;

    // Laser range finder ======================

    // Initialization
    lrf = new lrf_controller();

    fg_acquiring = false;

    // COM port
    for (int i = 1; i <= 20; i++)
        ui->comboBox_lrf_com->addItem("COM" + QString::number(i));

    // Baud rate
    QStringList list_baudrate;
    list_baudrate << "9600" << "19200" << "38400" << "76800" << "115200";
    ui->comboBox_lrf_baudRate->addItems(list_baudrate);

    // default settings
    ui->comboBox_lrf_com->setCurrentText("COM7");
    ui->comboBox_lrf_baudRate->setCurrentText("38400");
    lrf_status = false;
    lrfResetData();
    lrf_timer = new QTimer;
    connect(lrf_timer, SIGNAL(timeout()), this, SLOT(lrfReadData()));

    // Laser range finder ====================== End

    // Stereo vision ===========================

    // Initialization
    sv = new stereo_vision();

    fg_capturing = false;

    // COM port
    for (int i = 0; i < 5; i++) {
        ui->comboBox_cam_com_L->addItem(QString::number(i));
        ui->comboBox_cam_com_R->addItem(QString::number(i));
    }

    // focal length
    ui->comboBox_camera_focal_length->addItem("16");
    ui->comboBox_camera_focal_length->addItem("12");
    ui->comboBox_camera_focal_length->addItem("4");

    // base line
    ui->lineEdit_base_line->setText(QString::number(15.0));

    // default settings
    ui->comboBox_cam_com_L->setCurrentIndex(0);
    ui->comboBox_cam_com_R->setCurrentIndex(2);
    ui->comboBox_camera_focal_length->setCurrentIndex(0);

    // Stereo vision =========================== End

    // Thread control ==========================
    sync.addFuture(f_sv);
    sync.addFuture(f_lrf);
    // ========================================= End

    // camera calibration ======================
    // ========================================= End
}

MainWindow::~MainWindow()
{
#ifdef debug_info_main
    qDebug()<<"destructor";
#endif

    closeEvent(0);
    cv::destroyAllWindows();
    lrf->close();
    delete lrf;
    lrf_timer->stop();
    delete lrf_timer;
    delete sv;
    delete ui;
}

void MainWindow::reportError(QString part, QString level, QString content)
{
    ui->system_log->append("[" + part + "] " + level + "  " + content);
    QMessageBox::information(0, level, content);
}

void MainWindow::report(QString content)
{
    ui->system_log->append(content);
}

void MainWindow::on_pushButton_lrf_open_clicked()
{
    if (!lrf->open(ui->comboBox_lrf_com->currentText(), ui->comboBox_lrf_baudRate->currentText().toInt())) {
        reportError("lrf", "Error!", "Port cannot be open or under using.");
        return;
    }
    else {
        report("Port's opened.");
        lrf_timer->start(350);
    }
}

void MainWindow::lrfResetData()
{
    // reset data
    for (int i = 0; i < LENGTH_DATA; i++)
        lrf_data[i] = -1.0; //**// lrf range?
}

void MainWindow::lrfReadData()
{
    lrfResetData();

    if (lrf->acquireData(lrf_data) && lrf_status == false) {
        report("data: acquired");
        lrf_status = true;
    }
    else {
        report("data: lost");
        lrf_status = false;
    }

    // display
    cv::Mat display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);
    double angle = 0.0;

    for (int i = 0; i < LENGTH_DATA; ++i) {
        double r = lrf_data[i];
        double x = r * cos(angle * CV_PI / 180.0);
        double y = r * sin(angle * CV_PI / 180.0);

        cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, y / ui->spinBox_lrf_scale->value() + 100), 1, cv::Scalar(0, 0, 255), -1);
        if (LENGTH_DATA / 2 == i)
            cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, y / ui->spinBox_lrf_scale->value() + 100), 5, cv::Scalar(0, 255, 0), -1);

        angle += RESOLUTION;
    }

    cv::imshow("image", display_lrf);
    cv::waitKey(1);

}

void MainWindow::on_pushButton_lrf_display_clicked()
{
    lrfReadData();
}

void MainWindow::camOpen()
{
    int L = ui->comboBox_cam_com_L->currentIndex();
    int R = ui->comboBox_cam_com_R->currentIndex();

    if (!sv->open(L, R)) {
        reportError("sv", "Error!", "Camera can NOT be opened.");
        sv->close();
        return;
    }
    else {
        report("Port's opened.");
    }
}

void MainWindow::displaying(const cv::Mat &img_L, const cv::Mat &img_R, const cv::Mat &disp)
{
#ifdef debug_info_sv
    qDebug()<<"disp"<<&img_L;
#endif
    ui->label_cam_img_L->setPixmap(QPixmap::fromImage(QImage::QImage(img_L.data, img_L.cols, img_L.rows, 3 * img_L.cols, QImage::Format_RGB888)).scaled(IMG_W, IMG_H));
    ui->label_cam_img_R->setPixmap(QPixmap::fromImage(QImage::QImage(img_R.data, img_R.cols, img_R.rows, 3 * img_R.cols, QImage::Format_RGB888)).scaled(IMG_W, IMG_H));
    ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp.data, disp.cols, disp.rows, disp.cols, QImage::Format_Indexed8)).scaled(IMG_W, IMG_H));
    qApp->processEvents();
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

    camCapture();
}

void MainWindow::on_pushButton_cam_capture_clicked()
{
    if (!sv->isOpened()) {
        reportError("sv", "Error!", "Cameras haven't opened.");
        return;
    }

    fg_capturing = true;
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_cam_stop_clicked()
{
    fg_capturing = false;
}

void MainWindow::closeEvent(QCloseEvent *)
{
    // releas form since it's allocated
    if (fg_form_created)
        delete form_calib;
    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_capturing = false;
    fg_acquiring = false;
}

void MainWindow::threadProcessing()
{
    fg_running = true;
    while (fg_capturing | fg_acquiring) {
        // sv
        if (fg_capturing) {
            f_sv = QtConcurrent::run(sv, &stereo_vision::stereoVision);
            sync.setFuture(f_sv);
        }

        // lrf
        if (fg_acquiring) {

        }

        sync.waitForFinished();

        if (fg_capturing) {
            displaying(sv->img_r_L, sv->img_r_R, sv->disp);
        }

        if (fg_acquiring) {

        }
    }
    fg_running = false;
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
    form_calib = new calibrationForm();
    fg_form_created = true;
    form_calib->move(1500, 400);
    form_calib->show();

    QObject::connect(form_calib, SIGNAL(requestImage(char)), this, SLOT(requestImage(char)));
    QObject::connect(this, SIGNAL(sendImage(cv::Mat)), form_calib, SLOT(saveImage(cv::Mat)));
    QObject::connect(this, SIGNAL(sendImages(cv::Mat, cv::Mat)), form_calib, SLOT(saveImages(cv::Mat, cv::Mat)));
}

void MainWindow::requestImage(const char &CCD)
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
        emit sendImage(sv->img_r_L);
        break;
    case 'R':
#ifdef debug_info_cc
        qDebug()<<"send right image";
#endif
        emit sendImage(sv->img_r_R);
        break;
    case 'B':
#ifdef debug_info_cc
        qDebug()<<"send both images";
#endif
        emit sendImages(sv->img_r_L, sv->img_r_R);
        break;
    }

}
