#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    fg_running = false;

    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<QVector<int>>("QVector<int>");

    // Laser range finder ======================

    // Initialization
    lrf = new lrf_controller();

    fg_lrf_record = false;
    fg_lrf_record_quit = false;

    fg_acquiring = false;
    fg_buffering = false;

    QObject::connect(this, SIGNAL(lrfUpdateGUI()), this, SLOT(lrfDisplay()));

    ui->label_lrf_data->setStyleSheet("background-color:silver");

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

    // 3D display
    display_lrf_3D.clear();
    lrf_temp.clear();

    // Laser range finder ====================== End

    // Stereo vision ===========================

    // Initialization
    sv = new stereo_vision();

    fg_capturing = false;

    on_checkBox_pseudo_color_clicked(ui->checkBox_pseudo_color->isChecked());
    on_checkBox_sv_topview_clicked(ui->checkBox_sv_topview->isChecked());
    on_checkBox_sv_reproject_clicked(ui->checkBox_sv_reproject->isChecked());
    on_checkBox_topview_plot_points_clicked(ui->checkBox_topview_plot_points->isChecked());

    QObject::connect(sv, SIGNAL(svUpdateGUI(cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, int)), this, SLOT(svDisplay(cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, int)));

    ui->label_cam_img_L->setStyleSheet("background-color:silver");
    ui->label_cam_img_R->setStyleSheet("background-color:silver");
    ui->label_disp->setStyleSheet("background-color:silver");
    ui->label_sv_detected->setStyleSheet("background-color:silver");

    // COM port
    for (int i = 0; i < 5; i++) {
        ui->comboBox_cam_device_index_L->addItem(QString::number(i));
        ui->comboBox_cam_device_index_R->addItem(QString::number(i));
    }

    // focal length
    ui->comboBox_camera_focal_length->addItem("16");
    ui->comboBox_camera_focal_length->addItem("12");
    ui->comboBox_camera_focal_length->addItem("4");

    // top view
    svDisplayTopViewBG();
    // Stereo vision =========================== End

    // Radar ESR ===============================
    rc = new RadarController(0.0);

    radarDisplayTopViewBG();

    fg_retrieving = false;

    model_radar = new QStandardItemModel(64, 1, this);
    model_radar->setHorizontalHeaderItem(0, new QStandardItem(QString("Range")));
    ui->tableView_radar->setModel(model_radar);
    for (int i = 0 ; i < 64; i++) {
        model_radar->setItem(i, 0, &rc->item[i]);
    }

    on_checkBox_radar_topview_clicked(ui->checkBox_radar_topview->isChecked());

    QObject::connect(rc, SIGNAL(radarUpdateGUI(int, cv::Mat *, cv::Mat *)), this, SLOT(radarDisplay(int, cv::Mat *, cv::Mat *)));
    // Radar ESR =============================== End

    // Fusion ==================================
    initialFusedTopView();
    // Fusion ================================== End

    // Thread control ==========================
    f_sv.setPaused(true);
    f_lrf.setPaused(true);
    f_lrf_buf.setPaused(true);
    f_radar.setPaused(true);
    f_fused.setPaused(true);
    sync.addFuture(f_sv);
    sync.addFuture(f_lrf);
    sync.addFuture(f_lrf_buf);
    sync.addFuture(f_radar);
    sync.addFuture(f_fused);
    // ========================================= End

    // camera calibration ======================
    fg_form_calib_alloc = false;
    // ========================================= End

    // Stereo vision param =====================
    fg_form_smp_alloc = false;
    // ========================================= End

    // Pseudo color table ======================
    ui->label_color_table->setScaledContents(true) ;
    ui->label_color_table->setPixmap(QPixmap::fromImage(*sv->color_table));
    // ========================================= End

    // mouse control ===========================
//    QObject::connect(ui->label_cam_img_L, SIGNAL(mXY(int, int)), this, SLOT(mouseXY(int, int)));
    QObject::connect(ui->label_disp, SIGNAL(mXY(int, int)), this, SLOT(mouseXY(int, int)));
    // ========================================= End

    // default setting =========================
    fg_param_loaded = false;

    if (!projectFolder()) {
        reportError("path", "Warning.", "Path folder is NOT \"Fusion\".");
    }

    paramRead();

    // ========================================= End

    // Stereo vision (1) =======================
    // set status after loading params
    on_checkBox_do_calibration_clicked(ui->checkBox_do_calibration->isChecked());
    on_checkBox_do_depth_clicked(ui->checkBox_do_depth->isChecked());

#ifdef stereoMatchCuda
    on_radioButton_BM_clicked();
#endif
    // Stereo vision (1) ======================= End
}

MainWindow::~MainWindow()
{
#ifdef debug_info_main
    qDebug()<<"destructor";
#endif

    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
//    QMessageBox::StandardButton reply = QMessageBox::question(0, "New change", "Parameters were changed, save the new ones?", QMessageBox::Yes | QMessageBox::No);
//    if (reply == QMessageBox::Yes)
        paramWrite();
    cv::destroyAllWindows();

    delete rc;
    lrf->close();
    delete lrf;
    delete sv;
    delete[] sensors;
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *)
{
    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
    if (fg_form_calib_alloc)
        if (form_calib->isVisible())
            form_calib->close();
    if (fg_form_smp_alloc)
        if (form_smp->isVisible())
            form_smp->close();
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
//    std::cout<<ev->key()<<std::endl;
    int index = ui->tabWidget_display->currentIndex();
    int index_1 = ui->tabWidget->currentIndex();

    // combination clicks
    int key_in = ev->key();
    if (ev->modifiers() & Qt::ControlModifier) {
        ui->centralWidget->setFocus();
        key_in += Qt::CTRL;
//        std::cout<<key_in<<", "<<QKeySequence(key_in).toString(QKeySequence::NativeText).toStdString()<<std::endl;
        if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Up")) == QKeySequence::ExactMatch)
            ui->tabWidget_display->setCurrentIndex(index - 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Down")) == QKeySequence::ExactMatch)
            ui->tabWidget_display->setCurrentIndex(index + 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Left")) == QKeySequence::ExactMatch)
            ui->tabWidget->setCurrentIndex(index_1 - 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Right")) == QKeySequence::ExactMatch)
            ui->tabWidget->setCurrentIndex(index_1 + 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl++")) == QKeySequence::ExactMatch) {
            zoomInFusedTopView();
            updateFusedTopView();
        }
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+-")) == QKeySequence::ExactMatch) {
            zoomOutFusedTopView();
            updateFusedTopView();
        }
    }
}

MouseLabel::MouseLabel(QWidget * parent): QLabel(parent)
{
    this->setMouseTracking(true);
}

void MouseLabel::mouseMoveEvent(QMouseEvent *e)
{
    int x = e->x();
    int y = e->y();
    emit mXY(x, y);
}

void MainWindow::mouseXY(int x, int y)
{
    int xx, yy;
    xx = 2 * x;
    yy = 2 * y;
    lock.lockForRead();
    mouse_info.sprintf("(x,y) = (%d,%d), Disp. = %d, (X,Y,Z) = (%d,%d,%d)",
                       xx, yy, sv->data[yy][xx].disp,
                       -1, -1, sv->data[yy][xx].Z); //**// real X, Y, Z
    lock.unlock();
    ui->label_depth_info->setText(mouse_info);
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
    QFile fout(project_path.path() + "/basic_param.yml");
    if (!fout.exists()) {
        reportError("path", "Error!", "basic_param.yml is NOT existed.");
        return;
    }

    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::READ);

    cv::FileNode n;
    n = fs["stereoVision"];
    ui->comboBox_cam_device_index_L->setCurrentIndex((int) n["port_L"]);
    ui->comboBox_cam_device_index_R->setCurrentIndex((int) n["port_R"]);
    ui->comboBox_camera_focal_length->setCurrentIndex((int) n["cam_focal_length"]);
    sv->cam_param.cam_focal_length = ui->comboBox_camera_focal_length->currentText().toInt();
    sv->cam_param.base_line = (double) n["base_line"];
    sv->cam_param.focal_length = (double) n["focal_length"];
    ui->lineEdit_base_line->setText(QString::number(sv->cam_param.base_line));
    ui->label_sv_focal_length->setText(QString::number(sv->cam_param.focal_length));

    n = fs["topView"];
    ui->spinBox_topview_r->setValue((int) n["row_interval"]);
    ui->spinBox_topview_c->setValue((int) n["col_interval"]);
    ui->spinBox_radar_topview_r->setValue((int) n["row_interval_radar"]);
    ui->spinBox_radar_topview_c->setValue((int) n["col_interval_radar"]);

    n = fs["laserRangeFinder"];
    ui->comboBox_lrf_com->setCurrentIndex((int) n["port"]);
    ui->comboBox_lrf_baudRate->setCurrentIndex((int) n["baudRate"]);
    ui->spinBox_lrf_scale->setValue((double) n["mapScale"]);
    std::string res = (std::string) n["resolution"];
    if (res == "cm")
        ui->radioButton_lrf_res_cm->setChecked(true);
    else if (res == "mm")
        ui->radioButton_lrf_res_mm->setChecked(true);

    n = fs["stereoParamSGBM"];
    sv->param_sgbm.pre_filter_cap = (int)(n["pre_filter_cap"]);
    sv->param_sgbm.SAD_window_size = (int)(n["SAD_window_size"]);
    sv->param_sgbm.min_disp = (int)(n["min_disp"]);
    sv->param_sgbm.num_of_disp = (int)(n["num_of_disp"]);
    sv->param_sgbm.uniquenese_ratio = (int)(n["uniquenese_ratio"]);
    sv->param_sgbm.speckle_window_size = (int)(n["speckle_window_size"]);
    sv->param_sgbm.speckle_range = (int)(n["speckle_range"]);

    n = fs["stereoParamBM"];
    sv->param_bm.pre_filter_size = (int)(n["pre_filter_size"]);
    sv->param_bm.pre_filter_cap = (int)(n["pre_filter_cap"]);
    sv->param_bm.SAD_window_size = (int)(n["SAD_window_size"]);
    sv->param_bm.min_disp = (int)(n["min_disp"]);
    sv->param_bm.num_of_disp = (int)(n["num_of_disp"]);
    sv->param_bm.texture_thresh = (int)(n["texture_thresh"]);
    sv->param_bm.uniquenese_ratio = (int)(n["uniquenese_ratio"]);
    sv->param_bm.speckle_window_size = (int)(n["speckle_window_size"]);
    sv->param_bm.speckle_range = (int)(n["speckle_range"]);

    fs.release();
    fg_param_loaded = true;
}

void MainWindow::paramWrite()
{
    if (!fg_param_loaded)
        return;

    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::WRITE);

    fs << "stereoVision" << "{";
    fs << "port_L" << ui->comboBox_cam_device_index_L->currentIndex();
    fs << "port_R" << ui->comboBox_cam_device_index_R->currentIndex();
    fs << "cam_focal_length" << ui->comboBox_camera_focal_length->currentIndex();
    fs << "focal_length" << sv->cam_param.focal_length;
    fs << "base_line" << ui->lineEdit_base_line->text().toDouble();
    fs << "}";

    fs << "topView" << "{";
    fs << "row_interval" << ui->spinBox_topview_r->value();
    fs << "col_interval" << ui->spinBox_topview_c->value();
    fs << "row_interval_radar" << ui->spinBox_radar_topview_r->value();
    fs << "col_interval_radar" << ui->spinBox_radar_topview_c->value();
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

    fs << "stereoParamSGBM" << "{";
    fs << "pre_filter_cap" << sv->param_sgbm.pre_filter_cap;
    fs << "SAD_window_size" << sv->param_sgbm.SAD_window_size;
    fs << "min_disp" << sv->param_sgbm.min_disp;
    fs << "num_of_disp" << sv->param_sgbm.num_of_disp;
    fs << "uniquenese_ratio" << sv->param_sgbm.uniquenese_ratio;
    fs << "speckle_window_size" << sv->param_sgbm.speckle_window_size;
    fs << "speckle_range" << sv->param_sgbm.speckle_range;
    fs << "}";

    fs << "stereoParamBM" << "{";
    fs << "pre_filter_size" << sv->param_bm.pre_filter_cap;
    fs << "pre_filter_cap" << sv->param_bm.pre_filter_cap;
    fs << "SAD_window_size" << sv->param_bm.SAD_window_size;
    fs << "min_disp" << sv->param_bm.min_disp;
    fs << "num_of_disp" << sv->param_bm.num_of_disp;
    fs << "texture_thresh" << sv->param_bm.texture_thresh;
    fs << "uniquenese_ratio" << sv->param_bm.uniquenese_ratio;
    fs << "speckle_window_size" << sv->param_bm.speckle_window_size;
    fs << "speckle_range" << sv->param_bm.speckle_range;
    fs << "}";

    fs.release();
}

void MainWindow::initialFusedTopView()
{
    // vehicle & fused topview information
    detection_range_pixel = ui->label_fusion->width() / 2;

    detection_range = sv->max_distance;

    ratio = 1.0 * detection_range_pixel / sv->max_distance;

    vehicle.length = 400;

    vehicle.width = 180;

    // find max & min detection range in all sensors, so this function shall be called after initialization of all sensors
    max_detection_range = rc->max_distance;

    min_detection_range = rc->min_distance + sqrt(pow(0.5 * vehicle.length, 2) + pow(0.5 * vehicle.width, 2));

    fused_topview.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);
    fused_topview_BG.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);

    //**// You may CRASH if you add lots of sensors but you don't revise the number of the array
    // sensor information
    sensors = new sensorInformation[3];

    // sv
    sensors[0].color = cv::Scalar(255, 0, 0, 255);
    sensors[0].thickness = 2;

    // radar
    sensors[1].color = cv::Scalar(0, 0, 255, 255);
    sensors[1].thickness = 2;

    // object's label color
    QColor pic_color_sv = QColor(sensors[0].color[0], sensors[0].color[1], sensors[0].color[2]);
    QPixmap pic_sv(20, 20);
    pic_sv.fill(pic_color_sv);
    ui->label_fusion_sv_color->setPixmap(pic_sv);
    ui->label_fusion_sv_color->setPixmap(pic_sv);
    QColor pic_color_radar = QColor(sensors[1].color[0], sensors[1].color[1], sensors[1].color[2]);
    QPixmap pic_radar(20, 20);
    pic_radar.fill(pic_color_radar);
    ui->label_fusion_radar_color->setPixmap(pic_radar);

    updateFusedTopView();
}

void MainWindow::updateFusedTopView()
{
    ratio = 1.0 * detection_range_pixel / detection_range;

    vehicle.VCP = cv::Point(detection_range_pixel, detection_range_pixel);

    vehicle.rect = cv::Rect(vehicle.VCP.x - (int)(vehicle.width * ratio / 2), vehicle.VCP.y - (int)(vehicle.length * ratio / 2),
                            (int)(vehicle.width * ratio), (int)(vehicle.length * ratio));

    vehicle.color = cv::Scalar(0, 255, 0, 255);

    fused_topview.setTo(cv::Scalar(0, 0, 0, 0));

    fused_topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    // topview
    cv::circle(fused_topview_BG, vehicle.VCP, detection_range_pixel, rc->color_BG, -1, 8, 0);

    // max detection range [now]
    cv::putText(fused_topview_BG, QString::number((int)detection_range).toStdString() + " cm", cv::Point(vehicle.VCP.x + 0.65 * detection_range_pixel, vehicle.VCP.y  - 0.8 * detection_range_pixel), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, rc->color_line, 1);

    // 30 m
    float distance = sv->max_distance * ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // 5 m
    distance = 500 * ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // vehicle
    cv::rectangle(fused_topview_BG, vehicle.rect, vehicle.color, -1, 8, 0);

    ui->label_fusion_BG->setPixmap(QPixmap::fromImage(QImage::QImage(fused_topview_BG.data, fused_topview_BG.cols, fused_topview_BG.rows, QImage::Format_RGBA8888)));
    ui->label_fusion->setPixmap(QPixmap::fromImage(QImage::QImage(fused_topview.data, fused_topview.cols, fused_topview.rows, QImage::Format_RGBA8888)));

    // sensor information
    sensors[0].pos = cv::Point(0, vehicle.length * ratio / 2);

    sensors[1].pos = cv::Point(0, vehicle.length * ratio / 2);
}

void MainWindow::zoomOutFusedTopView()
{
    // zoom out (scrolling forward)
    if (detection_range == min_detection_range) {
        int count = min_detection_range / gap;
        if (count == 0)
            count = 1;
        detection_range = ceil(count) * gap;
    }
    else
        detection_range = detection_range + gap < max_detection_range ?
                    detection_range + gap : max_detection_range;
}

void MainWindow::zoomInFusedTopView()
{
    // zoom in (scrolling backward)
    if (detection_range == max_detection_range) {
        int count = max_detection_range / gap;
        detection_range = floor(count) * gap;
    }
    else
        detection_range = detection_range - gap > min_detection_range ?
                    detection_range - gap : min_detection_range;
}

void MainWindow::wheelEvent(QWheelEvent *ev)
{
    // vertical middle button
    if (ev->orientation() == Qt::Vertical) {
        if (ev->delta() > 0) {
            zoomOutFusedTopView();
        }
        if (ev->delta() < 0) {
            zoomInFusedTopView();
        }
    }

    updateFusedTopView();
}

void MainWindow::drawFusedTopView(stereo_vision::objInformation *d_sv, RadarController::ESR_track_object_info *d_radar)
{
    fused_topview.setTo(cv::Scalar(0, 0, 0, 0));

    int device = 0;
    if (ui->checkBox_fusion_sv->isChecked() && fg_capturing) {
        for (int k = 0; k < sv->obj_nums; k++) {
            if (d_sv[k].labeled) {
                cv::Point plot_pt;
                pointTransformTopView(sensors[device].pos, d_sv[k].range, d_sv[k].angle, &plot_pt);
                cv::circle(fused_topview, plot_pt, sensors[device].thickness, cv::Scalar(255, 0, 0, 255), -1, 8, 0);
            }
        }
    }

    device = 1;
    if (ui->checkBox_fusion_radar->isChecked() && fg_retrieving) {
        for (int m = 0; m < 64; m++) {
            if (d_radar[m].status >= rc->obj_status_filtered) {
                cv::Point plot_pt;
                pointTransformTopView(sensors[device].pos, 100 * d_radar[m].range, d_radar[m].angle + rc->aim_angle, &plot_pt);
                cv::circle(fused_topview, plot_pt, sensors[device].thickness, cv::Scalar(0, 0, 255, 255), -1, 8, 0);
            }
        }
    }

    ui->label_fusion->setPixmap(QPixmap::fromImage(QImage::QImage(fused_topview.data, fused_topview.cols, fused_topview.rows, QImage::Format_RGBA8888)));
}

void MainWindow::pointTransformTopView(cv::Point sensor_pos, float range, float angle, cv::Point *output)
{
    output->x = vehicle.VCP.x + (range * sin(angle * CV_PI / 180.0) * ratio + sensor_pos.x);
    output->y = vehicle.VCP.y - (range * cos(angle * CV_PI / 180.0) * ratio + sensor_pos.y);
}

void MainWindow::radarDisplayTopViewBG()
{
    if (rc->isInitializedTopView()) {
        rc->drawTopViewLines(ui->spinBox_radar_topview_r->value(), ui->spinBox_radar_topview_c->value(), false);
        ui->label_top_view_radar_long_BG->setPixmap(QPixmap::fromImage(QImage::QImage(rc->topview_BG.data, rc->topview_BG.cols, rc->topview_BG.rows, QImage::Format_RGBA8888)).scaled(900, 600));

        ui->label_radar_data_BG->setPixmap(QPixmap::fromImage(QImage::QImage(rc->img_radar_BG.data, rc->img_radar_BG.cols, rc->img_radar_BG.rows, QImage::Format_RGB888)));
    }
//    cv::imshow("rc", rc->topview_BG);
}

void MainWindow::on_spinBox_radar_topview_r_valueChanged(int arg1)
{
    radarDisplayTopViewBG();
}

void MainWindow::on_spinBox_radar_topview_c_valueChanged(int arg1)
{
    radarDisplayTopViewBG();
}

void MainWindow::svDisplayTopViewBG()
{
    if (sv->isInitializedTopView()) {
        sv->drawTopViewLines(ui->spinBox_topview_r->value(), ui->spinBox_topview_c->value(), true);
        ui->label_top_view_bg->setPixmap(QPixmap::fromImage(QImage::QImage(sv->topview_BG.data, sv->topview_BG.cols, sv->topview_BG.rows, QImage::Format_RGBA8888)).scaled(270, 750));
    }
}

void MainWindow::on_spinBox_topview_r_valueChanged(int arg1)
{
    svDisplayTopViewBG();
}

void MainWindow::on_spinBox_topview_c_valueChanged(int arg1)
{
    svDisplayTopViewBG();
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
        emit lrfUpdateGUI();
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

    ui->label_lrf_data->setPixmap(QPixmap::fromImage(QImage::QImage(display_lrf.data, display_lrf.cols, display_lrf.rows, 3 * display_lrf.cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    lock.unlock();
//    cv::imshow("image", display_lrf);
    cv::waitKey(10);

    if (fg_lrf_record) {
        lock.lockForWrite();
        for (int i = 0; i < LENGTH_DATA; i++) {
            fprintf(fp1, "%d ", lrf_data[i]); //**// %f? %d?
        }
        fprintf(fp1, "\n");
        lock.unlock();
        if (fg_lrf_record_quit) {
            fclose(fp1);
            fg_lrf_record = false;
        }
    }
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

void MainWindow::svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj)
{
    lock.lockForRead();
    ui->label_cam_img_L->setPixmap(QPixmap::fromImage(QImage::QImage(img_L->data, img_L->cols, img_L->rows, 3 * img_L->cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_cam_img_R->setPixmap(QPixmap::fromImage(QImage::QImage(img_R->data, img_R->cols, img_R->rows, 3 * img_R->cols, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_sv_detected_obj->setText(QString::number(detected_obj));

    if (ui->checkBox_do_depth->isChecked()) {    
        if (ui->checkBox_pseudo_color->isChecked())
            ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp_pseudo->data, disp_pseudo->cols, disp_pseudo->rows, QImage::Format_RGB888)).scaled(IMG_DIS_DISP_W, IMG_DIS_DISP_H));
        else
            ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp->data, disp->cols, disp->rows, disp->cols, QImage::Format_Indexed8)).scaled(IMG_DIS_DISP_W, IMG_DIS_DISP_H));

        // update topview
        if (ui->checkBox_sv_topview->isChecked()) {
            ui->label_top_view_sv->setPixmap(QPixmap::fromImage(QImage::QImage(topview->data, topview->cols, topview->rows, QImage::Format_RGBA8888)).scaled(270, 750));
            ui->label_sv_detected->setPixmap(QPixmap::fromImage(QImage::QImage(img_detected->data, img_detected->cols, img_detected->rows, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
        }
    }

    lock.unlock();
}

void MainWindow::on_checkBox_sv_topview_clicked(bool checked)
{
    if (checked) {
        sv->fg_topview = true;
        ui->checkBox_sv_reproject->setEnabled(true);
        for (int r = 0; r < sv->topview.rows; r++) {
            for (int c = 0; c < sv->topview.cols; c++) {
                cv::Vec4b val = sv->topview.at<cv::Vec4b>(r, c);
                if (val[0] != 0 || val [1] != 0 || val[2] != 0)
                    sv->topview.at<cv::Vec4b>(r, c)[3] = 255;
            }
        }
    }
    else {
        sv->fg_topview = false;
        ui->checkBox_sv_reproject->setEnabled(false);
        for (int r = 0; r < sv->topview.rows; r++) {
            for (int c = 0; c < sv->topview.cols; c++)
                sv->topview.at<cv::Vec4b>(r, c)[3] = 0;
        }
    }
    ui->label_top_view_sv->setPixmap(QPixmap::fromImage(QImage::QImage(sv->topview.data, sv->topview.cols, sv->topview.rows, QImage::Format_RGBA8888)).scaled(270, 750));
}

void MainWindow::on_pushButton_cam_open_clicked()
{
    camOpen();
    sv->input_mode = SV::INPUT_SOURCE::CAM;
    sv->matchParamInitialize(SV::STEREO_MATCH::SGBM);
    on_pushButton_cam_step_clicked();
}

void MainWindow::on_pushButton_cam_step_clicked()
{
    if (fg_capturing) {
        reportError("sv", "Warning!", "The frames are capturing now.");
        return;
    }

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

    while (!f_sv.isPaused()) {}
    threadCheck();
}

void MainWindow::threadCheck()
{
    if (fg_running && !fg_acquiring && !fg_buffering && !fg_capturing && !fg_retrieving)
        fg_running = false;
}

void MainWindow::threadBuffering()
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
            sv->stereoVision();
//            f_sv = QtConcurrent::run(sv, &stereo_vision::stereoVision);
            qApp->processEvents();
        }
        ui->label_sv_proc->setText(QString::number(t_proc.restart()));

        // lrf
        if (fg_acquiring && !f_lrf.isRunning()) {
//            lrfReadData();
            f_lrf = QtConcurrent::run(this, &MainWindow::lrfReadData);
            qApp->processEvents();
        }
        ui->label_lrf_proc->setText(QString::number(t_proc.restart()));

        // lrf buffer
        if (fg_buffering && lrf->bufNotFull() && !f_lrf_buf.isRunning()) {
//            lrf->pushToBuf();
            f_lrf_buf = QtConcurrent::run(lrf, &lrf_controller::pushToBuf);
            qApp->processEvents();
        }
        ui->label_lrf_buf_proc->setText(QString::number(t_proc.restart()));

        // Radar ESR
        if (fg_retrieving && !f_radar.isRunning()) {
            rc->retrievingData();
//            f_radar = QtConcurrent::run(rc, &RadarController::retrievingData);
            qApp->processEvents();
        }

        drawFusedTopView(sv->objects_display, rc->esr_obj);

        sync.waitForFinished();
    }

    sync.setCancelOnWait(true);
    while (!sync.cancelOnWait()) {}
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
        sv->cam_param.param_r = sv->cam_param.focal_length * sv->cam_param.base_line;
        sv->fg_stereoMatch = true;
    }
    else
        sv->fg_stereoMatch = false;
}

void MainWindow::on_checkBox_pseudo_color_clicked(bool checked)
{
    if (checked)
        sv->fg_pseudo = true;
    else
        sv->fg_pseudo = false;
}

void MainWindow::on_checkBox_sv_reproject_clicked(bool checked)
{
    if (checked)
        sv->fg_reproject = true;
    else
        sv->fg_reproject = false;
}

void MainWindow::on_checkBox_topview_plot_points_clicked(bool checked)
{
    if (checked)
        sv->fg_topview_plot_points = true;
    else
        sv->fg_topview_plot_points = false;
}

void MainWindow::on_pushButton_camera_calibration_clicked()
{
    if (fg_form_calib_alloc)
        return;
    form_calib = new calibrationForm();
    form_calib->move(1500, 500);
    fg_form_calib_alloc = true;
    QObject::connect(form_calib, SIGNAL(closed(void)), this, SLOT(closeFormCalib(void)));

    // send focal length & base line to form
    QObject::connect(this, SIGNAL(sendBasicInfo(int, double)), form_calib, SLOT(getBasicInfo(int, double)));
    QObject::connect(form_calib, SIGNAL(requestImage(char)), this, SLOT(requestImage(char)));
    QObject::connect(this, SIGNAL(sendImage(cv::Mat *)), form_calib, SLOT(saveImage(cv::Mat *)));
    QObject::connect(this, SIGNAL(sendImages(cv::Mat *, cv::Mat *)), form_calib, SLOT(saveImages(cv::Mat *, cv::Mat *)));

    emit sendBasicInfo(ui->comboBox_camera_focal_length->currentText().toInt(), ui->lineEdit_base_line->text().toDouble());

    form_calib->show();
}

void MainWindow::closeFormCalib(void)
{
    QObject::disconnect(form_calib, SIGNAL(closed(void)), this, SLOT(closeFormCalib(void)));
    delete form_calib;
    fg_form_calib_alloc = false;
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
    if (sv->match_mode == SV::STEREO_MATCH::BM)
        return;
    report("Change to BM mathod.");
    ui->radioButton_BM->setChecked(true);
    sv->matchParamInitialize(SV::STEREO_MATCH::BM);
    if (fg_form_smp_alloc) {
        sv->updateParamsSmp();
        form_smp->repaint();
    }
}

void MainWindow::on_radioButton_SGBM_clicked()
{
    if (sv->match_mode == SV::STEREO_MATCH::SGBM)
        return;
    report("Change to SGBM mathod.");
    sv->matchParamInitialize(SV::STEREO_MATCH::SGBM);
    if (fg_form_smp_alloc) {
        sv->updateParamsSmp();
        form_smp->repaint();
    }
}

void MainWindow::connectSmp(int old_mode, int new_mode)
{
    switch (old_mode) {
    case SV::STEREO_MATCH::BM:
        QObject::disconnect(form_smp, SIGNAL(send_bm_pre_filter_size(int)), sv, SLOT(change_bm_pre_filter_size(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_pre_filter_cap(int)), sv, SLOT(change_bm_pre_filter_cap(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_sad_window_size(int)), sv, SLOT(change_bm_sad_window_size(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_min_disp(int)), sv, SLOT(change_bm_min_disp(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_num_of_disp(int)), sv, SLOT(change_bm_num_of_disp(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_texture_thresh(int)), sv, SLOT(change_bm_texture_thresh(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_uniqueness_ratio(int)), sv, SLOT(change_bm_uniqueness_ratio(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_speckle_window_size(int)), sv, SLOT(change_bm_speckle_window_size(int)));
        QObject::disconnect(form_smp, SIGNAL(send_bm_speckle_range(int)), sv, SLOT(change_bm_speckle_range(int)));
        break;
#ifndef stereoMatchCuda
    case SV::STEREO_MATCH::SGBM:
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_pre_filter_cap(int)), sv, SLOT(change_sgbm_pre_filter_cap(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_sad_window_size(int)), sv, SLOT(change_sgbm_sad_window_size(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_min_disp(int)), sv, SLOT(change_sgbm_min_disp(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_num_of_disp(int)), sv, SLOT(change_sgbm_num_of_disp(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_uniqueness_ratio(int)), sv, SLOT(change_sgbm_uniqueness_ratio(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_speckle_window_size(int)), sv, SLOT(change_sgbm_speckle_window_size(int)));
        QObject::disconnect(form_smp, SIGNAL(send_sgbm_speckle_range(int)), sv, SLOT(change_sgbm_speckle_range(int)));
        break;
#endif
    }

    switch (new_mode) {
    case SV::STEREO_MATCH::BM:
        QObject::connect(form_smp, SIGNAL(send_bm_pre_filter_size(int)), sv, SLOT(change_bm_pre_filter_size(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_pre_filter_cap(int)), sv, SLOT(change_bm_pre_filter_cap(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_sad_window_size(int)), sv, SLOT(change_bm_sad_window_size(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_min_disp(int)), sv, SLOT(change_bm_min_disp(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_num_of_disp(int)), sv, SLOT(change_bm_num_of_disp(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_texture_thresh(int)), sv, SLOT(change_bm_texture_thresh(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_uniqueness_ratio(int)), sv, SLOT(change_bm_uniqueness_ratio(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_speckle_window_size(int)), sv, SLOT(change_bm_speckle_window_size(int)));
        QObject::connect(form_smp, SIGNAL(send_bm_speckle_range(int)), sv, SLOT(change_bm_speckle_range(int)));
        break;
#ifndef stereoMatchCuda
    case SV::STEREO_MATCH::SGBM:
        QObject::connect(form_smp, SIGNAL(send_sgbm_pre_filter_cap(int)), sv, SLOT(change_sgbm_pre_filter_cap(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_sad_window_size(int)), sv, SLOT(change_sgbm_sad_window_size(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_min_disp(int)), sv, SLOT(change_sgbm_min_disp(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_num_of_disp(int)), sv, SLOT(change_sgbm_num_of_disp(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_uniqueness_ratio(int)), sv, SLOT(change_sgbm_uniqueness_ratio(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_speckle_window_size(int)), sv, SLOT(change_sgbm_speckle_window_size(int)));
        QObject::connect(form_smp, SIGNAL(send_sgbm_speckle_range(int)), sv, SLOT(change_sgbm_speckle_range(int)));
        break;
#endif
    }

    form_smp->changeMode(new_mode);
}

void MainWindow::on_pushButton_stereo_match_param_clicked()
{
    if (fg_form_smp_alloc)
        return;
    form_smp = new stereoMatchParamForm(0, sv->match_mode);
    form_smp->move(1500, 100);
    fg_form_smp_alloc = true;
    QObject::connect(form_smp, SIGNAL(closed(void)), this, SLOT(closeFormSmp(void)));

    // send cuurent stereo matching params to ui
    QObject::connect(sv, SIGNAL(setConnect(int,int)), this, SLOT(connectSmp(int,int)));
    QObject::connect(sv, SIGNAL(sendCurrentParams(std::vector<int>)), form_smp, SLOT(updateParams(std::vector<int>)));
    sv->updateParamsSmp();

    form_smp->show();
}

void MainWindow::closeFormSmp(void)
{
    QObject::disconnect(form_smp, SIGNAL(closed(void)), this, SLOT(closeFormSmp(void)));
    QObject::disconnect(sv, SIGNAL(setConnect(int,int)), this, SLOT(connectSmp(int,int)));
    QObject::disconnect(sv, SIGNAL(sendCurrentParams(std::vector<int>)), form_smp, SLOT(updateParams(std::vector<int>)));
    delete form_smp;
    fg_form_smp_alloc = false;
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

void MainWindow::on_pushButton_lrf_request_ONCE_clicked()
{
    lrf->requestData(LRF::CAPTURE_MODE::ONCE);
    while (!lrf->bufEnoughSet()) {
        lrf->pushToBuf();
    }
    lrfReadData();
}

void MainWindow::on_pushButton_lrf_request_clicked()
{
    lrf->requestData(LRF::CAPTURE_MODE::CONTINUOUS);

    // push data to buffer //**// shouldn't be here
    fg_buffering = true;
    f_lrf_buf.setPaused(false);
//        lrf->bufRunning();
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_lrf_retrieve_clicked()
{
    fg_acquiring = true;
    f_lrf.setPaused(false);
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_lrf_stop_clicked()
{
    fg_buffering = false;
    fg_acquiring = false;
    f_lrf.setPaused(true);
    f_lrf_buf.setPaused(true);

    while (!(f_lrf.isPaused() && f_lrf_buf.isPaused())) {}
    lrf->stopRetrieve();

    threadCheck();

#ifdef debug_info_main
    qDebug()<<"stop done";
#endif
}

void MainWindow::on_lineEdit_sv_focal_length_returnPressed()
{
    sv->cam_param.focal_length = ui->lineEdit_sv_focal_length->text().toDouble();
    on_checkBox_do_depth_clicked(true);
}

void MainWindow::on_pushButton_lrf_record_data_clicked()
{
//    lrf_temp.clear();
//    for (int i = 0; i < LENGTH_DATA; i++) {
//        lrf_temp.push_back(lrf_data[i]);
//    }
//    display_lrf_3D.push_back(lrf_temp);

    fp1 = fopen("D_lrf.txt", "a");
    fg_lrf_record = true;
//    FILE* fp1;
//    fp1 = fopen("D_lrf.txt", "a");
//    for (int i = 0; i < LENGTH_DATA; i++) {
//        fprintf(fp1, "%f ", lrf_data[i]);
//    }
//    fprintf(fp1, "\n");
//    fclose(fp1);
}

void MainWindow::on_pushButton_sv_record_data_clicked()
{
    FILE* fp;
    fp = fopen("D_sv.txt", "w");

    for (int r = 0; r < IMG_H; r++) {
        for (int c = 0; c < IMG_W; c++) {
            fprintf(fp, "%d ", sv->data[r][c].disp);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);

    cv::Mat temp_L, temp_R;
    cv::cvtColor(sv->img_r_L, temp_L, cv::COLOR_RGB2BGR);
    cv::cvtColor(sv->img_r_R, temp_R, cv::COLOR_RGB2BGR);
    cv::imwrite("D_sv_L.jpg", temp_L);
    cv::imwrite("D_sv_R.jpg", temp_R);
}

void MainWindow::on_pushButton_lrf_record_stop_clicked()
{
    fg_lrf_record_quit = true;
}

void MainWindow::on_pushButton_sv_read_images_clicked()
{
    QStringList imgs_path = QFileDialog::getOpenFileNames(0, "Load images", ".", tr("Image files (*.jpg)"));
    if (imgs_path.empty())
        return;
    sv->input_mode = SV::INPUT_SOURCE::IMG;

    sv->img_L = cv::imread(imgs_path[0].toStdString());
    sv->img_R = cv::imread(imgs_path[1].toStdString());
    sv->stereoVision();
}

void MainWindow::readFromTxt(QString file_name, cv::Mat *output)
{
    QFile file(file_name);
    file.open(QIODevice::ReadOnly);
    if(!file.isOpen())
        return;
    QTextStream in(&file);

    std::vector <std::vector <short int> > img_d;
    std::vector <short int> temp;
    QString content, value;
    while (!in.atEnd()) {
        temp.clear();
        content.clear();
        content = file.readLine();
        for (int i = 0; i < content.size(); i++) {
            QChar va = content.at(i);
            if (va != ' ' && va != '\t')
                value.append(va);
            else {
                temp.push_back(value.toInt());
                value.clear();
            }
        }
        img_d.push_back(temp);
    }
    file.close();

    int row = img_d.size();
    int col = img_d[0].size();
    output->create(row, col, CV_16S);
    for (int r = 0; r < row; r++) {
        short int* ptr = output->ptr<short int>(r);
        for (int c = 0; c < col; c++) {
            ptr[c] = img_d[r][c];
//            std::cout<<ptr[c]<<" ";
        }
//        std::cout<<std::endl;
    }
}

void MainWindow::on_pushButton_sv_read_disp_clicked()
{
    cv::Mat img_sv;
    cv::Mat img_sv_r;
    cv::Mat hist = cv::Mat::zeros(1, 3000, CV_32S);
    cv::Mat hist_id = cv::Mat(1, 3000, CV_16S);
    for (short int i = 0; i < 3000; i++)
        hist_id.ptr<short int>(0)[i] = i;
    QString file_name = "D_sv.txt";
    readFromTxt(file_name, &img_sv);

    QTime countt;
    countt.start();
    // display
    img_sv_r.create(img_sv.rows, img_sv.cols, CV_8UC1);
    for (int r = 0; r < img_sv.rows; r++) {
        uchar* ptr_1 = img_sv_r.ptr<uchar>(r);
        short int* ptr = img_sv.ptr<short int>(r);
        for (int c = 0; c < img_sv.cols; c++) {
            int va = ptr[c];
            int va_1 = 1.0 * va / 8.0;
            if (va_1 >= 256)
                ptr_1[c] = 255;
            else if (va_1 <= 0)
                ptr_1[c] = 0;
            else
                ptr_1[c] = va_1;
            if (va == -16) {
                hist.ptr<long int>(0)[0]++;
            }
            else {
                hist.ptr<long int>(0)[va]++;
            }
//            std::cout<<ptr[c]<<" ";
        }
//        std::cout<<std::endl;
    }
    cv::imshow("[sv] new", img_sv_r);
    cv::imwrite("D_sv_disp.jpg", img_sv_r);

    // sorting
    cv::Mat hist_sort = hist.clone();
    for (int i = 0; i < hist_sort.cols - 2; i++) {
        for (int j = i + 1; j < hist_sort.cols - 1; j++) {
            if (hist_sort.ptr<long int>(0)[i] < hist_sort.ptr<long int>(0)[j]) {
                long int tempp = hist_sort.ptr<long int>(0)[j];
                hist_sort.ptr<long int>(0)[j] = hist_sort.ptr<long int>(0)[i];
                hist_sort.ptr<long int>(0)[i] = tempp;

                int tempp1 = hist_id.ptr<short int>(0)[j];
                hist_id.ptr<short int>(0)[j] = hist_id.ptr<short int>(0)[i];
                hist_id.ptr<short int>(0)[i] = tempp1;
            }
            continue;
        }
    }

//    int img_total = 0;
//    for (int i = 0; i < 3000; i++) {
//        img_total += hist_sort.ptr<long int>(0)[i];
//    }

    cv::Mat img_sv_separate;
    img_sv_separate.create(img_sv.rows, img_sv.cols, CV_8UC1);
    for (int k = 0; k < 3000; k++) {
        img_sv_separate.setTo(0);
        for (int r = 0; r < img_sv.rows; r++) {
            uchar* ptr_1 = img_sv_separate.ptr<uchar>(r);
            short int* ptr = img_sv.ptr<short int>(r);
            for (int c = 0; c < img_sv.cols; c++) {
                int dev = 20;
                if ((int)(ptr[c]) >= (int)(hist_id.ptr<short int>(0)[k]) - dev &&
                        (int)(ptr[c]) <= (int)(hist_id.ptr<short int>(0)[k]) + dev &&
                        hist_id.ptr<short int>(0)[k] >= 20 && hist_id.ptr<short int>(0)[k] < 1800 &&
                        hist_sort.ptr<long int>(0)[k] > 300) {
                    ptr_1[c] = 255;//1.0 * ptr[c] / 8.0;
                }
            }
        }
        QString name = "D_sv_disp_" + QString::number(k) + ".jpg";
        cv::imshow(name.toStdString(), img_sv_separate);
        static char c;
        c = cv::waitKey(0);
        if (c == 'q') {
            cv::destroyAllWindows();
            break;
        }
        else if (c == 's')
            cv::imwrite(name.toStdString(), img_sv_separate);
    }
}

void MainWindow::on_pushButton_lrf_read_range_clicked()
{
    cv::Mat img_lrf;
    cv::Mat img_lrf_r;
    QString file_name = "D_lrf.txt";
    readFromTxt(file_name, &img_lrf);
    cv::transpose(img_lrf, img_lrf);
    cv::resize(img_lrf, img_lrf, cv::Size(img_lrf.cols, (int)(2.2 * img_lrf.rows)));
    img_lrf_r.create(img_lrf.rows, img_lrf.cols, CV_8UC1);
    for (int r = 0; r < img_lrf.rows; r++) {
        uchar* ptr_1 = img_lrf_r.ptr<uchar>(r);
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            ptr_1[c] = 1.0 * ptr[c] / 35.0;
        }
    }
    cv::imshow("[lrf] new", img_lrf_r);
    cv::imwrite("D_lrf.jpg", img_lrf_r);

    cv::Mat img_lrf_proc = cv::Mat::zeros(img_lrf.rows, img_lrf.cols, CV_8UC1);
    int pxl_dev = 1;
    for (int r = 0; r < img_lrf.rows - pxl_dev; r++) {
        short int* ptr = img_lrf.ptr<short int>(r);
        uchar* ptr_p = img_lrf_proc.ptr<uchar>(r);
        for (int c = 0; c < img_lrf.cols - pxl_dev; c++) {
            int neighbor = ptr[c + pxl_dev];
            int smaller = ptr[c] > neighbor ? neighbor : ptr[c];
            int bigger = ptr[c] > neighbor ? ptr[c] : neighbor;
            double ratio = 1.0 * (bigger - smaller) / smaller;
            neighbor = img_lrf.ptr<short int>(r + pxl_dev)[c];
            smaller = ptr[c] > neighbor ? neighbor : ptr[c];
            bigger = ptr[c] > neighbor ? ptr[c] : neighbor;
            double ratio_1 = 1.0 * (bigger - smaller) / smaller;
            if (ratio > 0.5 || ratio_1 > 0.5) {
                ptr_p[c] = 255;
            }
//            std::cout<<ratio<<" ";
        }
//        std::cout<<std::endl;
    }

    cv::imshow("proc lrf", img_lrf_proc);
    cv::imwrite("D_lrf_edge.jpg", img_lrf_proc);
}

void MainWindow::on_pushButton_lrf_read_range_2_clicked()
{
    cv::Mat img_lrf;
    cv::Mat img_lrf_r;
    QString file_name = "D_lrf.txt";
    readFromTxt(file_name, &img_lrf);
    cv::transpose(img_lrf, img_lrf);
    cv::resize(img_lrf, img_lrf, cv::Size(img_lrf.cols * 2, img_lrf.rows * 2));
    img_lrf_r.create(img_lrf.rows, img_lrf.cols, CV_8UC1);
    double minn = 10000.0;
    double maxx = 0.0;
    for (int r = 0; r < img_lrf.rows; r++) {
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            if (ptr[c] < minn)
                minn = ptr[c];
            if (ptr[c] > maxx)
                maxx = ptr[c];
        }
    }
    for (int r = 0; r < img_lrf.rows; r++) {
        uchar* ptr_1 = img_lrf_r.ptr<uchar>(r);
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            int p = ptr[c];
            double va = ((255.0 * (p - minn)) / (1.0 * (maxx - minn)));
            if (va > 255.0)
                va = 255;
            else if (va < 0.0)
                va = 0;
            ptr_1[c] = (int)(va);
//            std::cout<<va<<"/"<<p<<" ";

        }
//        std::cout<<std::endl;
    }
//    std::cout<<std::endl<<minn<<", "<<maxx<<std::endl;

    cv::imshow("[lrf] new", img_lrf_r);
    cv::imwrite("D_lrf.jpg", img_lrf_r);
    cv::Mat img_lrf_edge;
    cv::Canny(img_lrf_r, img_lrf_edge, ui->horizontalSlider->value(), ui->horizontalSlider_2->value());
    cv::imshow("edge", img_lrf_edge);
    cv::imwrite("D_lrf_edge.jpg", img_lrf_edge);
}

void MainWindow::on_horizontalSlider_2_sliderReleased()
{
    on_pushButton_lrf_read_range_2_clicked();
}

void MainWindow::on_horizontalSlider_sliderReleased()
{
    on_pushButton_lrf_read_range_2_clicked();
}

void MainWindow::on_pushButton_lrf_request_2_clicked()
{
    lrf->requestData(LRF::CAPTURE_MODE::CONTINUOUS);

    fg_buffering = true;
    fg_acquiring = true;
}

void MainWindow::on_pushButton_lrf_retrieve_2_clicked()
{
    while (fg_acquiring) {
        lrfReadData();
        lrf->pushToBuf();

        qApp->processEvents();
    }
}

void MainWindow::on_pushButton_lrf_stop_2_clicked()
{
    fg_buffering = false;
    fg_acquiring = false;

    lrf->stopRetrieve();
    threadCheck();
}

void MainWindow::on_pushButton_radar_open_clicked()
{
    if (rc->open())
        report("Radar's Opened");
    else
        reportError("radar", "Error!", "Failed to open");
}

void MainWindow::on_pushButton_radar_write_clicked()
{
    if (rc->write())
        report("send cmd complete, data start!");
    else
        reportError("radar", "Error!", "Failed to send cmd to radar.");
}

void MainWindow::on_pushButton_radar_bus_on_clicked()
{
    rc->busOn();

    fg_retrieving = true;
    f_radar.setPaused(false);
    if (!fg_running)
        threadProcessing();
}

void MainWindow::on_pushButton_radar_bus_off_clicked()
{
    fg_retrieving = false;
    f_radar.setPaused(true);
    while (!f_radar.isPaused()) {}

    rc->busOff();
    threadCheck();
}

void MainWindow::radarDisplay(int detected_obj, cv::Mat *img, cv::Mat *topview)
{
    ui->label_radar_detected_obj->setText(QString::number(detected_obj));

    ui->label_radar_data->setPixmap(QPixmap::fromImage(QImage::QImage(img->data, img->cols, img->rows, QImage::Format_RGBA8888)));

    // update topview
    if (ui->checkBox_radar_topview->isChecked() && rc->current_count >= rc->update_count) {
        lock.lockForRead();
        ui->label_top_view_radar_long->setPixmap(QPixmap::fromImage(QImage::QImage(topview->data, topview->cols, topview->rows, QImage::Format_RGBA8888)).scaled(900, 600));
        lock.unlock();
    }
}

void MainWindow::on_checkBox_radar_topview_clicked(bool checked)
{
    if (checked) {
        rc->fg_topview = true;
        for (int r = 0; r < rc->topview.rows; r++) {
            for (int c = 0; c < rc->topview.cols; c++) {
                cv::Vec4b val = rc->topview.at<cv::Vec4b>(r, c);
                if (val[0] != 0 || val [1] != 0 || val[2] != 0)
                rc->topview.at<cv::Vec4b>(r, c)[3] = 255;
            }
        }
    }
    else {
        rc->fg_topview = false;
        for (int r = 0; r < rc->topview.rows; r++) {
            for (int c = 0; c < rc->topview.cols; c++)
                rc->topview.at<cv::Vec4b>(r, c)[3] = 0;
        }
    }
    ui->label_top_view_radar_long->setPixmap(QPixmap::fromImage(QImage::QImage(rc->topview.data, rc->topview.cols, rc->topview.rows, QImage::Format_RGBA8888)).scaled(900, 600));
}

void MainWindow::on_pushButton_stop_all_clicked()
{
    on_pushButton_cam_stop_clicked();
    on_pushButton_radar_bus_off_clicked();
}
