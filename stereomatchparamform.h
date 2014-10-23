#ifndef STEREOMATCHPARAMFORM_H
#define STEREOMATCHPARAMFORM_H

#include <QWidget>

#include "debug_info.h"

namespace Ui {
class stereoMatchParamForm;
}

class stereoMatchParamForm : public QWidget
{
    Q_OBJECT

public:
    explicit stereoMatchParamForm(QWidget *parent = 0);
    ~stereoMatchParamForm();

private slots:
    void on_horizontalSlider_pre_filter_size_valueChanged(int value);

    void on_horizontalSlider_pre_filter_cap_valueChanged(int value);

    void on_horizontalSlider_sad_window_size_valueChanged(int value);

    void on_horizontalSlider_min_disp_valueChanged(int value);

    void on_horizontalSlider_num_of_disp_valueChanged(int value);

    void on_horizontalSlider_texture_thresh_valueChanged(int value);

    void on_horizontalSlider_uniqueness_ratio_valueChanged(int value);

    void on_horizontalSlider_speckle_window_size_valueChanged(int value);

    void on_horizontalSlider_speckle_range_valueChanged(int value);

private:
    Ui::stereoMatchParamForm *ui;

signals:
    void send_pre_filter_size(int value);

    void send_pre_filter_cap(int value);

    void send_sad_window_size(int value);

    void send_min_disp(int value);

    void send_num_of_disp(int value);

    void send_texture_thresh(int value);

    void send_uniqueness_ratio(int value);

    void send_speckle_window_size(int value);

    void send_speckle_range(int value);
};

#endif // STEREOMATCHPARAMFORM_H
