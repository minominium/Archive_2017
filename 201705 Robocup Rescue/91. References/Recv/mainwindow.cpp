#include "mainwindow.h"
#include "ui_mainwindow.h"

#define RX_WAIT     0
#define RX_START    1
#define RX_GETDATA  2

#define DATA_LENGTH 13


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    // ComboBox 중 Baudrate 항목을 추가해준다.
    ui->ComboBox_Baudrate->addItem("4800");
    ui->ComboBox_Baudrate->addItem("9600");
    ui->ComboBox_Baudrate->addItem("19200");
    ui->ComboBox_Baudrate->addItem("38400");
    ui->ComboBox_Baudrate->addItem("57600");
    ui->ComboBox_Baudrate->addItem("115200");
    ui->ComboBox_Baudrate->addItem("230400");

    isSerialOpen = false;

    isSerialParsing = false;

    //ui->Button_Send->setEnabled(false);


    //SeriaPort(연결된 port)만큼 메모리 동적할당
    mSerialPort = new QSerialPort(this);

    //사용가능한 port를 탐색해서 list에 추가한다.
    QList<QSerialPortInfo> serialPortInfoList = QSerialPortInfo :: availablePorts();
    // ComboBox 중 Port항목에 현재 사용가능한 port의 이름을 추가한다.
    foreach(const QSerialPortInfo &serialPortInfo, serialPortInfoList)
    {
        ui->ComboBox_Port->addItem(serialPortInfo.portName());
    }


    // Timer 기능을 쓰고싶으면 맨 아래 Timer 함수에 내용을 적어주면 됨.
    mTimer = new QTimer(this);
    mTimer ->start(5);
    // Timer의 신호를 SLOT의 괄호 안에 있는 함수로 보내준다.(SLOT안에 있는 함수를 발생시키겠다.)
    connect(mTimer,SIGNAL(timeout()),this,SLOT(qtTimer()));


    // IP 주소를 받아올 변수를 선언.
    QString ipAddress;
    // 모든 IP 주소를 LIST에 저장
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();

    // LIST의 size만큼 반복 (192. 로 시작하는 wifi의 ip주소값을 탐색)
    for (int i = 0; i < ipAddressesList.size(); ++i)
    {
        if (ipAddressesList.at(i) != QHostAddress::LocalHost &&
            ipAddressesList.at(i).toIPv4Address())
        {
            if (ipAddressesList.at(i).toString().toStdString().find("192") != std::string::npos)
            {
                ipAddress = ipAddressesList.at(i).toString();
            }
        }
    }
    // 192가 탐색이 되지 않은 경우 Localhost에 연결( wi-fi 가 아닌 랜선인 경우 127로 시작하기 때문에 랜선의 ip를 찾겠다 라는 뜻)
    if (ipAddress.isEmpty())
        ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
    // text 입력 란에 ip와 port를 설정
    ui->textIP->setText(ipAddress);
    ui->textPort->setText("8258");

    bool portOK = false;
    // mAddress 변수를 현재 탐색된 ipAddress로 설정
    mAddress.setAddress(ipAddress);
    mPort = ui->textPort->text().toInt(&portOK,10);

    ui->cameraViewer_1->setScaledContents(true);
    ui->cameraViewer_2->setScaledContents(true);
    ui->cameraViewer_3->setScaledContents(true);
    ui->cameraViewer_4->setScaledContents(true);
    isServerOpen = false;
    rm.rotate(180);

}

MainWindow::~MainWindow()
{
    mSerialPort->close();
    delete ui;
}

void MainWindow::readImageData()
{
    QByteArray buffer;
    qint64 length = mUDPSocket->bytesAvailable();
    buffer.resize(length);

    QHostAddress senderAddress;
    quint16 senderPort;


    mUDPSocket->readDatagram(buffer.data(),buffer.size(),&senderAddress,&senderPort);

    if(senderAddress != mAddress)
    {
        int camRxBuf[3]={0,};

        camRxBuf[0]=(int)buffer.data()[0];
        if(camRxBuf[0]==0)return;
        camRxBuf[1]=(int)buffer.data()[1];
        camRxBuf[2]=(int)buffer.data()[2];

        // 영상을 받아올 때 각 영상의 끝을 알리는 숫자를 함께 보내게 되는데
        // 이 숫자가 일치하는 경우 그 숫자를 지워주고 JPG형태로 압축하겠다는 뜻.
        if(camRxBuf[0]*2==camRxBuf[1]
        &&camRxBuf[0]*3==camRxBuf[2])
        {
            QByteArray data = buffer.remove(0,3);

            QPixmap pixmap = QPixmap::fromImage(QImage::fromData((uchar *)data.data(),data.size(),"JPG"));
            this->setCameraPixmap(camRxBuf[0]-1,pixmap);
        }
    }
}

void MainWindow::setCameraPixmap(int index, QPixmap &pixmap)
{
        // index는 카메라 번호가 되고 카메라 번호에 맞는 영상을 띄어준다.
         switch(index)
        {
        case 0:
            pixmap = pixmap.transformed(rm);
             ui->cameraViewer_1->setPixmap(pixmap);
            break;
        case 1:
            ui->cameraViewer_2->setPixmap(pixmap);
            break;
        case 2:

            ui->cameraViewer_3->setPixmap(pixmap);

            break;
        case 3:
            ui->cameraViewer_4->setPixmap(pixmap);

            break;
        default:
            printf("setCameraPixmap : Camera index Error\n");
            break;
        }

}
void MainWindow::on_buttonOpen_clicked()
{
    bool portOK = false;

    mPort = ui->textPort->text().toInt(&portOK,10);

    mUDPSocket = new QUdpSocket(this);
    if(mUDPSocket->bind(QHostAddress::Any,mPort))
    {
        connect(mUDPSocket,SIGNAL(readyRead()),this,SLOT(readImageData()));
        ui->buttonOpen->setEnabled(false);
        ui->textIP->setEnabled(false);
        ui->textPort->setEnabled(false);

        isServerOpen = true;

        QMessageBox::information(this,"server","server open");
    }
    else
    {
        isServerOpen = false;
        QMessageBox::critical(this,"error","open error");
    }
}

void MainWindow::on_Button_Connect_clicked()
{
    if(isSerialOpen){
        // serial disconnect
        QMessageBox::about(this,"serial","close");
        isSerialOpen = false;
        ui->ComboBox_Port->setEnabled(true);
        ui->ComboBox_Baudrate->setEnabled(true);
        ui->Button_Connect->setText("connect");
        //ui->Button_Send->setEnabled(false);
        QMessageBox::about(this,"serial","close");
        mSerialPort -> close();
    }

    else
    {
        try{

            // serial connect
            bool ok;
            mPort1 = ui -> ComboBox_Port->currentText(); // mPort에 저장
            mBaudrate = ui -> ComboBox_Baudrate->currentText().toInt(&ok,10);

            if(mPort1 == "select" || ok == false)
                throw (QString)"Error";

            mSerialPort->setBaudRate(mBaudrate);
            mSerialPort->setPortName(mPort1);
            mSerialPort->setDataBits(QSerialPort::Data8);
            mSerialPort->setParity(QSerialPort::NoParity);
            mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

            if(mSerialPort->open(QIODevice::ReadWrite))
            {
                isSerialOpen = true;

                ui->ComboBox_Port->setEnabled(false);
                ui->ComboBox_Baudrate->setEnabled(false);
                ui->Button_Connect->setText("disconnect");
                ui->statusBar->showMessage(mPort1 + " : " + ui->ComboBox_Baudrate->currentText(),1000);
                //ui->Button_Send->setEnabled(true);
                QMessageBox::about(this,"serial","open\n");
            }

            else
            {
                throw (QString)"comport Error";
            }

            connect(mSerialPort,SIGNAL(readyRead()),this,SLOT(readData()));
        }
        catch(const QString str){
            QMessageBox::critical(this,"error","Error");
        }
    }
}

void MainWindow::on_Button_refresh_clicked()
{
    ui->ComboBox_Port->clear();
    ui->ComboBox_Port->addItem("select");

    QList<QSerialPortInfo> serialPortInfoList = QSerialPortInfo :: availablePorts();

    foreach(const QSerialPortInfo &serialPortInfo, serialPortInfoList)
    {
        ui->ComboBox_Port->addItem(serialPortInfo.portName());
    }
}
void MainWindow::readData()
{
    // recvBuffer 에 받아온 조종기 값을 저장
    QByteArray recvBuffer = mSerialPort->readAll();
    int size = recvBuffer.size();
    //    rxState  = RX_WAIT;
    //    rxBufCnt = 0;
    //    rxCheckSum = 0;
    isOut = false;

    //모든 조종기 값이 들어올 때까지 반복 (뿌리지 않고 저장)
    for(int i = 0 ; i< recvBuffer.size() ; i++)
    {
        unsigned char rxData = (char)recvBuffer[i];

        switch(rxState)
        {
            case RX_WAIT:
            if(rxData==0xAA)rxState = RX_START;
            //std::cout << "case : 1" << std::endl;
            break;

        case RX_START:
            if(rxData == 0xFF)rxState =  RX_GETDATA;
            //std::cout << "case : 2" << std::endl;
            break;

        case RX_GETDATA:

            //printf("%d\n",(int)rxData);
            rxBuf[rxBufCnt++] = rxData;
            //std::cout << "case : 3" << std::endl;
            if(rxBufCnt>DATA_LENGTH-1)
            {
                for (rxIdx = 0 ; rxIdx<DATA_LENGTH-1; rxIdx++)
                rxCheckSum += rxBuf[rxIdx];

                    if (rxCheckSum == rxBuf[DATA_LENGTH-1])
                    {

                        isOut = true;
                        for( int j= 0 ;j<DATA_LENGTH;j++)
                        {
                            recvRealData[j+2] = rxBuf[j];   //except 2 id, save data
                        }

                        QByteArray byteArray;
                        QBuffer buffer(&byteArray);
                        byteArray = QByteArray((char*)recvRealData, 15);

                        // server 와 serial이 연결된 상태에서 받은 조종기 값을 broadcast(같은 와이파이에 연결된 모든 컴퓨터)에 뿌린다.
                        if(isServerOpen&&isSerialOpen)
                        {
                            mUDPSocket->writeDatagram(buffer.data(),buffer.size(),QHostAddress::Broadcast,mPort);
                        }
                    }

                    rxState  = RX_WAIT;
                    rxBufCnt = 0;
                    rxCheckSum = 0;
            }
            break;

            default:
            break;

        }
           if(isOut==true)break;
    }
}
void MainWindow::qtTimer()
{
}
