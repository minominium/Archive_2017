#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace cv;

VideoCapture cap;
Mat matCap;
QString result = "";
QImage qImageCap;
QZXing decoder;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // start timer when button "Start" clicked
    connect(mTimer, SIGNAL(timeout()), this, SLOT(processFrameUpdateGUI()));
    decoder.setDecoder(QZXing::DecoderFormat_QR_CODE | QZXing::DecoderFormat_EAN_13);
    mTimer->start(0);

    ui->setupUi(this);
    ui->label->setScaledContents(true);
    ui->btnStop->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// Camera refresh function
void MainWindow::processFrameUpdateGUI()
{
    cap.read(matCap);

    // Break when no data
    if(matCap.empty())
    {
        cout << "Empty Image" << endl;
        return;
    }
    else
    {
        // Set Camera image on label
        cvtColor(matCap, matCap, COLOR_BGR2RGB);
        qImageCap = QImage((uchar*)matCap.data, matCap.cols, matCap.rows, matCap.step, QImage::Format_RGB888);
        ui->label->setPixmap(QPixmap::fromImage(qImageCap));

        // Qr Code decoding & print result
        if(decoder.decodeImage(qImageCap) != 0)
        {
            result = decoder.decodeImage(qImageCap);
            ui->qrLabel->setText(result);
        }
    }

}

void MainWindow::on_btnStart_clicked()
{
    ui->btnStop->setEnabled(true);
    ui->btnStart->setEnabled(false);

    cap.open(1);

    // Break when cannot open Camera
    if(!cap.isOpened())
    {
        return;
    }

    // Restart when start -> stop -> start
    if(!mTimer->isActive())
    {
        mTimer->start(0);
    }
}

void MainWindow::on_btnStop_clicked()
{
    ui->btnStop->setEnabled(false);
    ui->btnStart->setEnabled(true);
    mTimer->stop();
}
