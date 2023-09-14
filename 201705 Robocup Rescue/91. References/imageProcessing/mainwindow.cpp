#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    int check = 0;
    std::string file1 = "C:\\Users\\minom\\Desktop\\QrTest.png";
    std::string file2 = "C:\\Users\\minom\\Desktop\\QrTest2.jpg";
    pictureCompare(file1, file2, check);
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::pictureCompare(std::string imgF, std::string imgS, int checking)
{

    Mat img_object = imread(imgF, CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_scene = imread(imgS, CV_LOAD_IMAGE_GRAYSCALE);

    if(!img_object.data || !img_scene.data)
    {
        printf("Error!");
        return -1;
    }
    if(!img_object.data || !img_scene.data)
    {
        printf("--(!) Error reading images\n");
        return -1;
    }

    //Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector(minHessian);
    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect(img_object, keypoints_1);
    detector.detect(img_scene, keypoints_2);

    //Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;

    extractor.compute(img_object, keypoints_1, descriptors_1);
    extractor.compute(img_scene, keypoints_2, descriptors_2);

    //Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    double max_dist = 0, min_dist = 100;

    //Quick calculation of max and min distance between keypoints
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist)
        {
            min_dist = dist;
        }
        if(dist > max_dist)
        {
            max_dist = dist;
        }
    }

    printf("Max dist : %f\n", max_dist);
    printf("Min dist : %f\n", min_dist);

    std::vector <DMatch> good_matches;

    for(int i = 0; i < descriptors_1.rows; i++)
    {
        if(matches[i].distance <= max(2 * min_dist, 0.02))
        {
            checking++;
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_matches;
    drawMatches(img_object, keypoints_1, img_scene, keypoints_2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Good Matches", img_matches);

    for(int i = 0; i < (int)good_matches.size(); i++)
    {
        printf("Good Match[%d] Keypoint 1: %d, Keypoint 2: %d\n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
    }

    if(checking > 50)
    {
        printf("Same Picture\n");
    }
    else
    {
        printf("Different Picture\n");
    }

    waitKey(0);
    return checking;
}

void MainWindow::readme()
{
    printf("Usage: ./SURF_FlannMatcher <img1> <img2>\n");
}



void MainWindow::on_pushButtonShutDown_clicked()
{
    cv::destroyAllWindows();
    this -> close();
}
