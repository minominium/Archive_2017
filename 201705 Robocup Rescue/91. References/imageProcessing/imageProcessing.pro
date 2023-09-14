#-------------------------------------------------
#
# Project created by QtCreator 2017-01-10T19:26:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = imageProcessing
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
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += C:\OpenCV_MinGW\include

LIBS += C:\OpenCV_MinGW\bin\libopencv_core2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_highgui2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_features2d2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_calib3d2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_contrib2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_imgproc2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_nonfree2410.dll
LIBS += C:\OpenCV_MinGW\bin\libopencv_objdetect2410.dll


#INCLUDEPATH += D:\opencv\build\include
#INCLUDEPATH += D:\opencv_contrib\modules\xfeatures2d\include

#LIBS += D:\opencv-build\bin\libopencv_core331.dll
#LIBS += D:\opencv-build\bin\libopencv_highgui331.dll
#LIBS += D:\opencv-build\bin\libopencv_imgcodecs331.dll
#LIBS += D:\opencv-build\bin\libopencv_imgproc331.dll
#LIBS += D:\opencv-build\bin\libopencv_features2d331.dll
#LIBS += D:\opencv-build\bin\libopencv_calib3d331.dll
#LIBS += D:\opencv-build\bin\libopencv_videoio331.dll
#LIBS += D:\opencv-build\bin\libopencv_video331.dll
