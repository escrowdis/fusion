#include "stereomatchparamform.h"
#include "ui_stereomatchparamform.h"

stereoMatchParamForm::stereoMatchParamForm(QWidget *parent, int mode) :
    QWidget(parent),
    ui(new Ui::stereoMatchParamForm)
{
    this->mode = mode;
    ui->setupUi(this);
    ui->tabWidget->setCurrentIndex(mode);
}

stereoMatchParamForm::~stereoMatchParamForm()
{
    delete ui;
}

void stereoMatchParamForm::closeEvent(QCloseEvent *)
{
    emit closed();
}

void stereoMatchParamForm::changeMode(int mode)
{
    this->mode = mode;
}

void stereoMatchParamForm::updateParams(std::vector<int> param)
{
    switch (mode) {
    case SV::STEREO_MATCH::BM:
        ui->horizontalSlider_bm_pre_filter_size->setValue(param[0]);
        ui->horizontalSlider_bm_pre_filter_cap->setValue(param[1]);
        ui->horizontalSlider_bm_sad_window_size->setValue(param[2]);
        ui->horizontalSlider_bm_min_disp->setValue(param[3]);
        ui->horizontalSlider_bm_num_of_disp->setValue(param[4]);
        ui->horizontalSlider_bm_texture_thresh->setValue(param[5]);
        ui->horizontalSlider_bm_uniqueness_ratio->setValue(param[6]);
        ui->horizontalSlider_bm_speckle_window_size->setValue(param[7]);
        ui->horizontalSlider_bm_speckle_range->setValue(param[8]);
        break;
    case SV::STEREO_MATCH::SGBM:
        ui->horizontalSlider_sgbm_pre_filter_cap->setValue(param[0]);
        ui->horizontalSlider_sgbm_sad_window_size->setValue(param[1]);
        ui->horizontalSlider_sgbm_min_disp->setValue(param[2]);
        ui->horizontalSlider_sgbm_num_of_disp->setValue(param[3]);
        ui->horizontalSlider_sgbm_uniqueness_ratio->setValue(param[4]);
        ui->horizontalSlider_sgbm_speckle_window_size->setValue(param[5]);
        ui->horizontalSlider_sgbm_speckle_range->setValue(param[6]);
        break;
    }
}

void stereoMatchParamForm::on_horizontalSlider_bm_pre_filter_size_valueChanged(int value)
{
    if (value % 2 == 0) {
        value++;
        ui->horizontalSlider_bm_pre_filter_size->setValue(value);
    }
    emit send_bm_pre_filter_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_pre_filter_cap_valueChanged(int value)
{
    emit send_bm_pre_filter_cap(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_sad_window_size_valueChanged(int value)
{
    if (value % 2 == 0) {
        value++;
        ui->horizontalSlider_bm_sad_window_size->setValue(value);
    }
    emit send_bm_sad_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_min_disp_valueChanged(int value)
{
    emit send_bm_min_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_num_of_disp_valueChanged(int value)
{
    if (value % 16 != 0)
        value += (16 - value % 16);
    ui->horizontalSlider_bm_num_of_disp->setValue(value);
    emit send_bm_num_of_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_texture_thresh_valueChanged(int value)
{
    emit send_bm_texture_thresh(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_uniqueness_ratio_valueChanged(int value)
{
    emit send_bm_uniqueness_ratio(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_speckle_window_size_valueChanged(int value)
{
    emit send_bm_speckle_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_bm_speckle_range_valueChanged(int value)
{
    emit send_bm_speckle_range(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_pre_filter_cap_valueChanged(int value)
{
    emit send_sgbm_pre_filter_cap(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_sad_window_size_valueChanged(int value)
{
    if (value % 2 == 0) {
        value++;
        ui->horizontalSlider_sgbm_sad_window_size->setValue(value);
    }
    emit send_sgbm_sad_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_min_disp_valueChanged(int value)
{
    emit send_sgbm_min_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_num_of_disp_valueChanged(int value)
{
    if (value % 16 != 0)
        value += (16 - value % 16);
    ui->horizontalSlider_sgbm_num_of_disp->setValue(value);
    emit send_sgbm_num_of_disp(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_uniqueness_ratio_valueChanged(int value)
{
    emit send_sgbm_uniqueness_ratio(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_speckle_window_size_valueChanged(int value)
{
    emit send_sgbm_speckle_window_size(value);
}

void stereoMatchParamForm::on_horizontalSlider_sgbm_speckle_range_valueChanged(int value)
{
    emit send_sgbm_speckle_range(value);
}
