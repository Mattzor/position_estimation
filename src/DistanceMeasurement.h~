#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include<unistd.h>


using namespace cv;
using namespace std;

#ifndef DistanceMeasurement_H
#define DistanceMeasurement_H



class DistanceMeasurement{
   public:
      float focalLength, truckAngle, globalAngle;
      VideoCapture cap;
      void openCap();
      void closeCap();
      void pTransform(Mat frame,Mat &result);
      void open();
      void linePointSettings();
      void setContours(Mat clonedImage, Mat & cannyOut, vector<vector<Point> > &contours);
      void drawLines(Mat &cameraFeed, vector<vector<Point> > contours);
      void calculateDis(Point a, Point b, Point c);
      float scale = 335.0 / 720.0;
 
   private:
      int WIDTH = 640;
      int HEIGHT = 480;
      Point startPoint, move;
      Point truckPos;
      const float SCALE = 0.0025;
      int edges = 3;
      int startX = 100;
      int startY = 250;
      int roiWidth = 450;
      int roiHeight = 200;
      bool triangleOk(vector<Point> points);
      bool setLenOfTri(vector<Point> sidesOfTri);
      float lengthOfPoints(Point a, Point b);
      bool sizeOfTriangle(Point a, Point b, Point c);
      float rotationAngle(Point c, Point midOfAB);
      float truckSignAngle(Point midOfTri, Point truckPos, Point c, Point midOfAB, float angle);
      int triangleSizeDiff = 40;
      int standardLength = 20;
      Mat scale_frame;   
   
};


#endif 
