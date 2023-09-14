#include "mainwindow.h"
#include "ui_mainwindow.h"

#define CVX_YELLOW CV_RGB(255,255,0)
#define CVX_WHITE CV_RGB(255,255,255)
#define CVX_RED CV_RGB(255,0,0)
#define phi2xy(mat) cvPoint(cvRound(img -> width/2 + img -> width/3*cos(mat -> data.fl[0])), cvRound(img -> height/2 - img -> width/3*sin(mat -> data.fl[0])))

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui -> setupUi(this);

    // 초기화
    cvNamedWindow("Kalman", 1);
    CvRandState rng;
    cvRandInit(&rng, 0, 1, -1, CV_RAND_UNI);
    IplImage* img = cvCreateImage(cvSize(500, 500), 8, 3);
    CvKalman* kalman = cvCreateKalman(2, 1, 0);

    CvMat* x_k = cvCreateMat(2, 1, CV_32FC1);                   // 상태 벡터 x_k 초기화는 각위치와 각속도
    cvRandSetRange(&rng, 0, 0.1, 0);                            // 난수이용 초기화
    rng.disttype = CV_RAND_NORMAL;
    cvRand(&rng, x_k);

    CvMat* w_k = cvCreateMat(2, 1, CV_32FC1);                   // 프로세스 잡음

    // 측정 벡터는 한 개다
    CvMat* z_k = cvCreateMat(1, 1, CV_32FC1);
    cvZero(z_k);

    // 전이 행렬 F(시간이 k일 때와 k + 1 일 때 상태 사이의 관계를 규정)
    const float F[] = {1, 1, 0, 1};
    memcpy(kalman -> transition_matrix -> data.fl, F, sizeof(F));

    // 추가 초기화
    cvSetIdentity(kalman -> measurement_matrix, cvRealScalar(1));
    cvSetIdentity(kalman -> process_noise_cov, cvRealScalar(1e-5));
    cvSetIdentity(kalman -> measurement_noise_cov, cvRealScalar(1e-1));
    cvSetIdentity(kalman -> error_cov_post, cvRealScalar(1));

    cvRand(&rng, kalman -> state_post);                         // 초기 상태 임의로 결정

    while(1)
    {
        const CvMat* y_k = cvKalmanPredict(kalman, 0);         // 점 위치 예측
        cvRandSetRange(&rng, 0, sqrt(kalman -> measurement_noise_cov -> data.fl[0]), 0);        //측정치 생성 (z_k)
        cvRand(&rng, z_k);
        cvMatMulAdd(kalman -> measurement_matrix, x_k, z_k, z_k);

        // 결과 : 점으로 표시
        cvZero(img);
        cvCircle(img, phi2xy(z_k), 4, CVX_YELLOW);              // 새로운 측정치
        cvCircle(img, phi2xy(y_k), 4, CVX_WHITE, 2);            // 추정 예측상태
        cvCircle(img, phi2xy(x_k), 4, CVX_RED);                 // 실제 움직임 상태
        cvShowImage("Kalman", img);

        cvKalmanCorrect(kalman, z_k);

        cvRandSetRange(&rng, 0, sqrt(kalman -> process_noise_cov -> data.fl[0]), 0);
        cvRand(&rng, w_k);
        cvMatMulAdd(kalman -> transition_matrix, x_k, w_k, x_k);

        if(cvWaitKey(100) == 27)
        {
            break;
        }
    }

    // 종료 처리
    cvDestroyWindow("Kalman");
    cvReleaseImage(&img);
    cvReleaseKalman(&kalman);
    cvReleaseMat(&x_k);
    cvReleaseMat(&w_k);
    cvReleaseMat(&z_k);
}

MainWindow::~MainWindow()
{
    delete ui;
}
