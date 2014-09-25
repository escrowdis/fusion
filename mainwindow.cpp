#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Basic parameters
    // COM port
    for (int i = 1; i <= 10; i++)
        ui->comboBox_com->addItem("COM" + QString::number(i));

    // Baud rate
    QStringList list_baudrate;
    list_baudrate << "9600" << "19200" << "38400" << "76800" << "115200";
    ui->comboBox_baudRate->addItems(list_baudrate);

    // default settings
    ui->comboBox_com->setCurrentText("COM7");
    ui->comboBox_baudRate->setCurrentText("38400");

    lrf_status = false;

    resetData();

    timer = new QTimer;

    connect(timer, SIGNAL(timeout()), this, SLOT(readData()));
}

MainWindow::~MainWindow()
{
    cv::destroyAllWindows();
    lrf.close();
    timer->stop();
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    lrf.open(ui->comboBox_com->currentText(), ui->comboBox_baudRate->currentText().toInt());
    if (!lrf.isOpen()) {
        ui->system_log->append("Error!\tPort cannot be open or under using.");
        QMessageBox::information(0, "Error!", "Port cannot be open or under using.");
    }
    else {
        ui->system_log->append("Port opened.");
        timer->start(350);
    }
}

void MainWindow::resetData()
{
    // reset data
    for (int i = 0; i < length_data; i++)
        lrf_data[i] = -1.0; //**// lrf range?
}

void MainWindow::readData()
{
    resetData();

    if (lrf.acquireData(lrf_data) && lrf_status == false) {
        ui->system_log->append("data: acquired");
        lrf_status = true;
    }
    else {
        ui->system_log->append("data: lost");
        lrf_status = false;
    }

    // display
    cv::Mat display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);
    double angle = 0.0;

    for (int i = 0; i < length_data; ++i) {
        double r = lrf_data[i];
        double x = r * cos(angle * CV_PI / 180.0);
        double y = r * sin(angle * CV_PI / 180.0);

        cv::circle(display_lrf, cv::Point(x / ui->spinBox_scale->value() + 400, y / ui->spinBox_scale->value() + 100), 1, cv::Scalar(0, 0, 255), -1);
        if (length_data / 2 == i)
            cv::circle(display_lrf, cv::Point(x / ui->spinBox_scale->value() + 400, y / ui->spinBox_scale->value() + 100), 5, cv::Scalar(0, 255, 0), -1);

        angle += resolution;
    }

    cv::imshow("image", display_lrf);
    cv::waitKey(1);

}

void MainWindow::on_pushButton_2_clicked()
{
    readData();
}
