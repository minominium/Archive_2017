#ifndef ROBITVISION_H
#define ROBITVISION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;
/*////////////////////////////////////////////////////////////////////////////////////////////////////////*/

const double  _PI= 3.1415926535897932385;
const double _2PI= 6.283185307179586477;

typedef struct ColorInfoHsv{
     int hmin;
     int smin;
     int vmin;

     int hmax;
     int smax;
     int vmax;

     ColorInfoHsv()
     {
          hmin=0;
          smin=0;
          vmin=0;

          hmax=0;
          smax=0;
          vmax=0;
     }
}ColorInfoHsv;

/*////////////////////////////////////////////////////////////////////////////////////////////////////////*/

namespace RV
{
     void rangeFilter(const Mat& input,const Scalar& min,const Scalar& max,Mat& output);
     void rgb2hsv(const Mat& data_rgb, Mat &data_hsv);
     void DrawLine(Mat& img, const Point& PtA, const Point& PtB , const Scalar &color ,const int LineThresHold);
}

class RobitKalmanFilter {

private:
         double Q ;
         double R ;
         double X, P, K;

         void measurementUpdate();

public:
        RobitKalmanFilter(double initValue) {
            X = initValue;
            Q = 0.00002;
            R = 0.001;
            X = 0.0;
            P = 1.0;
            K = 0.0;
        }
        ~RobitKalmanFilter()
        {
        }
         const double update(double measurement);
};

class RobitRansacLine {

    struct sLine{
        double mx;
        double my;
        double sx;
        double sy;
    };

private:
    unsigned char* m_ImgData;
    int m_width;
    int m_height;


    double m_lineThreshold;
    double m_degree;

    Point m_line_A,m_line_B;
    vector<Point> m_pos;

    bool m_noSamples;

    void getSamples();
    void compute_model_parameter(const vector<Point>& vInlierPoint, sLine &model);
    double compute_distance(sLine &line, Point &x)const;

public:
    RobitRansacLine(const Mat& img);

    void runRansac();
    double getDegree()const{return m_degree;}

    Point getPointLineA()const{return m_line_A;}
    Point getPointLineB()const{return m_line_B;}
};

class RobitRansacCircle {

private:
      cv::Mat img_roi;

      unsigned char* m_ImgData;
      int m_width;
      int m_height;

      Point m_center;
      vector<Point> m_pos;

      double m_minRadius;
      double m_maxRadius;

      double m_cost;
      double m_radius;

      bool m_noSamples;

      void getSamples();
      bool getCircle(Point& p1,Point& p2,Point& p3, Point& center, double& radius)const;


      double verifyCircle(const Point& center, double radius)const;
public:

        RobitRansacCircle(const Mat& img);
        void runRansac();

        void setMinRadius(const double dist){m_minRadius=dist;}
        void setMaxRadius(const double dist){m_maxRadius=dist;}

        Point getPointCenter()const{return m_center;}
        double getCost()const{return m_cost;}
        double getRadius()const{return m_radius;}
};



#endif // ROBITVISION_H
