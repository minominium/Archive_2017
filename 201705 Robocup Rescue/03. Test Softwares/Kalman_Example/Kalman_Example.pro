#-------------------------------------------------
#
# Project created by QtCreator 2017-07-26T21:18:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Kalman_Example
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    cbloblabelingrobit.cpp \
    robitvision.cpp

HEADERS  += mainwindow.h \
    cbloblabelingrobit.h \
    robitvision.h

FORMS    += mainwindow.ui

INCLUDEPATH += C:/OpenCV_MinGW/include

LIBS += -LC:\OpenCV_MinGW\lib
LIBS += -lopencv_core2410
LIBS += -lopencv_imgproc2410
LIBS += -lopencv_highgui2410
LIBS += -lopencv_ml2410
LIBS += -lopencv_video2410
LIBS += -lopencv_features2d2410
LIBS += -lopencv_calib3d2410
LIBS += -lopencv_objdetect2410
LIBS += -lopencv_contrib2410
LIBS += -lopencv_legacy2410
LIBS += -lopencv_flann2410
LIBS += -lopencv_nonfree2410
