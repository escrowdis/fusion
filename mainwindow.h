#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>

#include <iostream>
#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;

    // Laser range finder =====
    lrf_controller lrf;

    double data[361];
    // ========================

};

#endif // MAINWINDOW_H
