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

    // reset data
    for (int i = 0; i < length_data; i++)
        data[i] = -1.0; //**// lrf range?
}

MainWindow::~MainWindow()
{
    lrf.close();
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
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    cv::Mat display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);
    if (lrf.acquireData(data))
        ui->system_log->append("data: acquired");
    else
        ui->system_log->append("data: lost");
}
