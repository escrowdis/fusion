#include "stereomatchparamform.h"
#include "ui_stereomatchparamform.h"

stereoMatchParamForm::stereoMatchParamForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::stereoMatchParamForm)
{
    ui->setupUi(this);
}

stereoMatchParamForm::~stereoMatchParamForm()
{
    delete ui;
}

void stereoMatchParamForm::on_horizontalSlider_pre_filter_size_valueChanged(int value)
{
    emit send_pre_filter_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_pre_filter_cap_valueChanged(int value)
{
    emit send_pre_filter_cap(value);
}

void stereoMatchParamForm::on_horizontalSlider_sad_window_size_valueChanged(int value)
{
    emit send_sad_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_min_disp_valueChanged(int value)
{
    emit send_min_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_num_of_disp_valueChanged(int value)
{
    emit send_num_of_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_texture_thresh_valueChanged(int value)
{
    emit send_texture_thresh(value);
}

void stereoMatchParamForm::on_horizontalSlider_uniqueness_ratio_valueChanged(int value)
{
    emit send_uniqueness_ratio(value);
}

void stereoMatchParamForm::on_horizontalSlider_speckle_window_size_valueChanged(int value)
{
    emit send_speckle_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_speckle_range_valueChanged(int value)
{
    emit send_speckle_range(value);
}
