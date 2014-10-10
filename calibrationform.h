#ifndef CALIBRATIONFORM_H
#define CALIBRATIONFORM_H

#include "debug_info.h"

#include <QWidget>
#include <QKeyEvent>

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

    void on_pushButton_calibration_clicked();

    void keyReleaseEvent(QKeyEvent *event);

signals:
    void saveImage();

private:
    Ui::calibrationForm *ui;

    bool fg_SaveBoth;           // check type of saving type -> true: Save both images false: Save single side
    char CCD;                   // Which CCD is on processing -> R, L

};

#endif // CALIBRATIONFORM_H
