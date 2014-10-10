#ifndef CALIBRATIONFORM_H
#define CALIBRATIONFORM_H

#include <QWidget>
#include "debug_info.h"

#include "camera_calibration.h"

namespace Ui {
class calibrationForm;
}

class calibrationForm : public QWidget
{
    Q_OBJECT

public:
    explicit calibrationForm(QWidget *parent = 0);
    ~calibrationForm();

    camera_calibration *cc;

private slots:
    void on_pushButton_3_clicked();

private:
    Ui::calibrationForm *ui;
};

#endif // CALIBRATIONFORM_H