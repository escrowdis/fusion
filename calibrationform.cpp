#include "calibrationform.h"
#include "ui_calibrationform.h"

calibrationForm::calibrationForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::calibrationForm)
{
    ui->setupUi(this);
    cc = new camera_calibration();
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
