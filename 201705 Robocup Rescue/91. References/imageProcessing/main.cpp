#include "mainwindow.h"
#include <QApplication>

#ifndef HAVE_OPENCV_NONFREE

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}



#endif
