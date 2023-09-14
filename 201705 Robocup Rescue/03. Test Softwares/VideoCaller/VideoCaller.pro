#-------------------------------------------------
#
# Project created by QtCreator 2017-12-27T14:03:46
#
#-------------------------------------------------


#-- include for qr decoding
include(QZXing/src/QZXing.pri)

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VideoCaller
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui

#-- OpenCV includes
INCLUDEPATH += D:\OpenCV\opencv\build\include
INCLUDEPATH += D:\OpenCV\opencv_contrib\modules\xfeatures2d\include

LIBS += D:\OpenCV\opencv-build\bin\libopencv_core340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_highgui340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_imgcodecs340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_imgproc340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_features2d340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_calib3d340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_videoio340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_video340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_xfeatures2d340.dll
LIBS += D:\OpenCV\opencv-build\bin\libopencv_flann340.dll
