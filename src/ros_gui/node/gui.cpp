#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
