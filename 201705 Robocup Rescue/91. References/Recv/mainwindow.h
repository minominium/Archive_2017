#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>

#include <QMessageBox>
#include <QtNetwork>
#include <QTimer>
#include <QUdpSocket>
#include <QPixmap>
#include <QFileDialog>
#include <iostream>

#include <QStringList>

#define RX_WAIT     0   //id waiting
#define RX_START    1   //id2 waiting => check beat
#define RX_GETDATA  2   //get other data

#define DATA_LENGTH 13
#define MAX_CAMERA_NUMBER   4

using namespace std;

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
    void on_buttonOpen_clicked();
    void readImageData();

    void on_Button_Connect_clicked();

    void on_Button_refresh_clicked();
    void qtTimer();

    void readData();

private:
    Ui::MainWindow *ui;


    // Server
    QUdpSocket* mUDPSocket;
    QHostAddress mAddress;
    quint16 mPort;
    bool isServerOpen;

    bool isSerialOpen;
    QString mPort1;
    int mBaudrate;
    unsigned char txBuf[15];

    unsigned char chkSum;
    bool isSerialParsing;

    QSerialPort* mSerialPort;
    QTimer *mTimer;
    bool isOut;
    QMatrix rm;
    int x  = 0;
    unsigned char recvBuffer[15];
    unsigned char cnt=0;
    int rxIdx = 0;
    unsigned char receivePoint  =0 ;
    unsigned int recved = 0;
    unsigned char recvRealData[15] = {0xAA,0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0};
    unsigned char rxState = RX_WAIT;
    unsigned char rxBuf[DATA_LENGTH];
    int rxBufCnt = 0 ;
    unsigned char rxCheckSum =0;
    // Camera View
    void setCameraPixmap(int index, QPixmap &pixmap);

};

#endif // MAINWINDOW_H
