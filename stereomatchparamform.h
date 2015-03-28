#ifndef STEREOMATCHPARAMFORM_H
#define STEREOMATCHPARAMFORM_H

#include <QWidget>

#include "debug_info.h"
#include <stereo_vision.h>

namespace Ui {
class stereoMatchParamForm;
}

class stereoMatchParamForm : public QWidget
{
    Q_OBJECT

public:
    explicit stereoMatchParamForm(QWidget *parent = 0, int mode = SV::STEREO_MATCH::SGBM);
    ~stereoMatchParamForm();

    void changeMode(int mode);

private slots:
    void on_horizontalSlider_bm_pre_filter_size_valueChanged(int value);

    void on_horizontalSlider_bm_pre_filter_cap_valueChanged(int value);

    void on_horizontalSlider_bm_sad_window_size_valueChanged(int value);

    void on_horizontalSlider_bm_min_disp_valueChanged(int value);

    void on_horizontalSlider_bm_num_of_disp_valueChanged(int value);

    void on_horizontalSlider_bm_texture_thresh_valueChanged(int value);

    void on_horizontalSlider_bm_uniqueness_ratio_valueChanged(int value);

    void on_horizontalSlider_bm_speckle_window_size_valueChanged(int value);

    void on_horizontalSlider_bm_speckle_range_valueChanged(int value);

    void closeEvent(QCloseEvent *);

    void on_horizontalSlider_sgbm_pre_filter_cap_valueChanged(int value);

    void on_horizontalSlider_sgbm_sad_window_size_valueChanged(int value);

    void on_horizontalSlider_sgbm_min_disp_valueChanged(int value);

    void on_horizontalSlider_sgbm_num_of_disp_valueChanged(int value);

    void on_horizontalSlider_sgbm_uniqueness_ratio_valueChanged(int value);

    void on_horizontalSlider_sgbm_speckle_window_size_valueChanged(int value);

    void on_horizontalSlider_sgbm_speckle_range_valueChanged(int value);

    void updateParams(int cur_mode, std::vector<int> param);

private:
    Ui::stereoMatchParamForm *ui;

    int mode;

    bool fg_sgbm_changed;

    bool fg_bm_changed;

signals:

    void closed(void);

    // BM ===========================
    void send_bm_pre_filter_size(int value);

    void send_bm_pre_filter_cap(int value);

    void send_bm_sad_window_size(int value);

    void send_bm_min_disp(int value);

    void send_bm_num_of_disp(int value);

    void send_bm_texture_thresh(int value);

    void send_bm_uniqueness_ratio(int value);

    void send_bm_speckle_window_size(int value);

    void send_bm_speckle_range(int value);
    // ============================== End

    // SGBM =========================
    void send_sgbm_pre_filter_cap(int value);

    void send_sgbm_sad_window_size(int value);

    void send_sgbm_min_disp(int value);

    void send_sgbm_num_of_disp(int value);

    void send_sgbm_uniqueness_ratio(int value);

    void send_sgbm_speckle_window_size(int value);

    void send_sgbm_speckle_range(int value);
    // ============================== End
};

#endif // STEREOMATCHPARAMFORM_H
