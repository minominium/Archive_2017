#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace cv;
using namespace std;

Mat objectHistogram;
Mat globalHistogram;

void getObjectHistogram(Mat &frame, Rect object_region);
void backProjection(const Mat &frame, const Mat &histogram, Mat &bp);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    VideoCapture cap(0);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    if(!cap.isOpened())
    {
        cout << "Cannot open Cam." << endl;
        return;
    }

    namedWindow("Configure the range to Find", CV_WINDOW_AUTOSIZE);

    int LowH = 170;
    int HighH = 179;

    int LowS = 50;
    int HighS = 255;

    int LowV = 0;
    int HighV = 255;

    cvCreateTrackbar("LowH", "Configure the range to Find", &LowH, 179);
    cvCreateTrackbar("HighH", "Configure the range to Find", &HighH, 179);

    cvCreateTrackbar("LowS", "Configure the range to Find", &LowS, 255);
    cvCreateTrackbar("HighS", "Configure the range to Find", &HighS, 255);

    cvCreateTrackbar("LowV", "Configure the range to Find", &LowV, 255);
    cvCreateTrackbar("HighV", "Configure the range to Find", &HighV, 255);

    Rect trackingWindow(0, 0, 30, 30);
    int i = 0;

    Rect prev_rect;
    Mat bp;

    while(true)
    {
        i++;

        int fps = cap.get(CV_CAP_PROP_FPS);
        cout << fps << " " << i << endl;

        Mat img_input, img_hsv, img_binary;

        bool ret = cap.read(img_input);

        if(!ret)
        {
            cout << "Cannot Open Image from Cam" << endl;
            break;
        }

        cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

        inRange(img_hsv, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), img_binary);

        erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        if(i < 180)
        {
            Mat img_labels, stats, centroids;
            int numOfLabels = connectedComponentsWithStats(img_binary, img_labels, stats, centroids, 8, CV_32S);

            int max = -1, idx = 0;
            for(int j = 1; j < numOfLabels; j++)
            {
                int area = stats.at<int>(j, CC_STAT_AREA);
                if(max < area)
                {
                    max = area;
                    idx = j;
                }
            }

            int left = stats.at<int>(idx, CC_STAT_LEFT);
            int top = stats.at<int>(idx, CC_STAT_TOP);
            int width = stats.at<int>(idx, CC_STAT_WIDTH);
            int height = stats.at<int>(idx, CC_STAT_HEIGHT);

            rectangle(img_input, Point(left, top), Point(left + width, top + height), Scalar(0, 0, 255), 1);

            Rect object_region(left, top, width, height);

            getObjectHistogram(img_hsv, object_region);
            prev_rect = object_region;
        }
        else
        {
            backProjection(img_hsv, objectHistogram, bp);
            bitwise_and(bp, img_binary, bp);
            RotatedRect rect  = CamShift(bp, prev_rect, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 1));

            ellipse(img_input, rect, CV_RGB(255, 0, 0), 3, CV_AA);
        }

        imshow("Binary Image", img_binary);
        imshow("Original Image", img_input);

        if(waitKey(1) == 27)
        {
            break;
        }
    }
    return;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnClose_clicked()
{
    this->close();
}

void getObjectHistogram(Mat &frame, Rect object_region)
{
    const int channels[] = {0, 1};
    const int histSize[] = {64, 64};
    float range[] = {0, 256};
    const float *ranges[] = {range, range};

    Mat objectROI = frame(object_region);
    calcHist(&objectROI, 1, channels, noArray(), objectHistogram, 2, histSize, ranges, true, false);

    calcHist(&frame, 1, channels, noArray(), globalHistogram, 2, histSize, ranges, true, true);

    for(int y = 0; y < objectHistogram.rows; y++)
    {
        for(int x = 0; x < objectHistogram.cols; x++)
        {
            objectHistogram.at<float>(y, x) /= globalHistogram.at<float>(y, x);
        }
    }
    normalize(objectHistogram, objectHistogram, 0, 255, NORM_MINMAX);
}

void backProjection(const Mat &frame, const Mat &histogram, Mat &bp)
{
    const int channels[] = {0, 1};
    float range[] = {0, 256};
    const float *ranges[] = {range, range};
    calcBackProject(&frame, 1, channels, objectHistogram, bp, ranges);
}

