#include "mainwindow.h"
#include <QApplication>

// thread control
#include <QReadWriteLock>
QReadWriteLock lock_sv;
QReadWriteLock lock_lrf;
QReadWriteLock lock_radar;
#include <QDir>
QDir project_path;
#include "recording/recording.h"
recording re;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
