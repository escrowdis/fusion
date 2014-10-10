#include "calibrationform.h"
#include "ui_calibrationform.h"

calibrationForm::calibrationForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::calibrationForm)
{
    ui->setupUi(this);
    cc = new camera_calibration(parent);

    QObject::connect(cc, SIGNAL(saveImage()), this, SLOT(doSaveImage()));
}

calibrationForm::~calibrationForm()
{
    delete cc;
    delete ui;
}

void calibrationForm::on_pushButton_3_clicked()
{
    close();
}

void calibrationForm::on_pushButton_calibration_clicked()
{
    cc->start();
}

void calibrationForm::doSaveImage()
{
    qDebug()<<"hi";
}
