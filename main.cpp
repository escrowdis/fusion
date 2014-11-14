#include "mainwindow.h"
#include <QApplication>

// thread control
#include <QReadWriteLock>
QReadWriteLock lock;
#include <QDir>
QDir project_path;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
