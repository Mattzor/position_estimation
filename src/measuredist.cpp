/*
Copyright (c) 2017, Robert Krook
Copyright (c) 2017, Erik Almblad
Copyright (c) 2017, Hawre Aziz
Copyright (c) 2017, Alexander Branzell
Copyright (c) 2017, Mattias Eriksson
Copyright (c) 2017, Carl Hjerpe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
Lane detection algorithm implemented in C++ with the OpenCV libarary: http://opencv.org

Requirements: OpenCV 3.1 (+),   C++11/gnu++11

Input: Video capture from camera device, resolution 640x360 

Output: Detected outer lanes and computed midlane overlayed on the original video. 

        Each lane represented by a set of two points. (x,y)

Commands: Press "ESC" to exit


        type following to compile the program 
        g++ -o main main.cpp `pkg-config opencv --cflags --libs`  -std=gnu++11

*/







#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
#include "laneDetection.h"
#include "DistanceMeasurement.h"
using namespace std;
using namespace cv;

/*
	type following to compile the program 
        g++ -o main main.cpp `pkg-config opencv --cflags --libs`  -std=gnu++11

*/

#define M_PI 3.14159265358979323846 
#define X1 0
#define X2 2
#define Y1 1
#define Y2 3
#define THICKNESS 10
#define BLUE Scalar(255,0,0)
#define BLACK Scalar(255,255,255)
#define WIDTH 640 
#define HEIGHT 480
#define PIX_PER_CM 15.37



/* linear regression function */


void LaneDetection::linearFit(double &slope, double &m, vector<int>& xValues, vector<int>& yValues)
{
    int i,k,n;
    n = xValues.size();
    double a, b;
    double xsum=0.0,x2sum=0.0,ysum=0.0,xysum=0.0;
    
    for (i=0;i<n;i++){
        xsum += xValues[i];                        //calculate sigma(xi)
        ysum += yValues[i];                        //calculate sigma(yi)
        x2sum += pow(xValues[i],2);                //calculate sigma(x^2i)
        xysum += xValues[i]*yValues[i];                    //calculate sigma(xi*yi)
    }

    a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);            //calculate slope
    b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);            //calculate intercept
    
    slope = a;
    m = b;
}



/* This function calculates slopes of lines passed in as argument */
void LaneDetection::findSlopes(vector<Vec4i> lines, double thresholdSlope, vector<float>& s, vector<Vec4i>& tmp){
   vector<float>slopes; 
   vector<Vec4i>lanesTemp;
   int x1, x2, y1, y2;
   float slope;
   for(Vec4i line: lines){
      x1 = line[X1]; y1 = line[Y1]; x2 = line[X2]; y2 = line[Y2]; 
      if(x2 == x1)
         x2 += 1;
      
      slope = float(y2 - y1)/(x2 - x1);
      if(abs(slope) > thresholdSlope){
	slopes.push_back(slope);
        lanesTemp.push_back(line);
      }
   }
   s = slopes;
   tmp = lanesTemp;
}


void LaneDetection::estimateOptimalLane(vector<Vec4i> lines, double& slope, double& lineConst, bool& drawLine){
    bool draw = true;
    vector<int> linesX, linesY;
    double line_slope, line_const;
    int x1, x2, y1, y2;
    for(Vec4i line : lines){
       x1 = line[X1]; y1 = line[Y1]; x2 = line[X2]; y2 = line[Y2];
       linesX.push_back(x1);
       linesX.push_back(x2);
       linesY.push_back(y1);
       linesY.push_back(y2);
    }
    
    if(linesX.size() > 0){    
      linearFit(line_slope, line_const, linesX, linesY);
   
    }else{
       line_slope = 1; 	
       line_const = 1;
       draw = false;
    }
    slope = line_slope;
    lineConst = line_const;
    drawLine = draw;
}




/* Function for drawing the lines on the screen */
void LaneDetection::drawLanes(Mat cameraFeed, vector<Vec4i> lines, double thresholdSlope, double trapH){
  bool draw_right = false;
  bool draw_left = false;
  vector<float>slopes;
  vector<Vec4i>lanesTemp, rightLines, leftLines;
  int x1, x2, y1, y2;
  
  findSlopes(lines, thresholdSlope, slopes, lanesTemp);
 
  int i =0;
  for(Vec4i line : lanesTemp){
     x1 = line[X1]; y1 = line[Y1]; x2 = line[X2]; y2 = line[Y2];
     float imgXCenter = cameraFeed.size().width / 2;
     if(slopes[i] > 0 && x1 > imgXCenter && x2 > imgXCenter){
        rightLines.push_back(line);
     }else if(slopes[i] < 0 && x1 < imgXCenter && x2 < imgXCenter){
        leftLines.push_back(line);
     }
     i++;
  }
  
  double right_slope, right_const, left_slope, left_const;
  estimateOptimalLane(rightLines, right_slope, right_const, draw_right);
  estimateOptimalLane(leftLines, left_slope, left_const, draw_left);
  y1 = int(cameraFeed.size().height);	
  y2 = int(cameraFeed.size().height * (1- trapH));
 
  int right_x1 = int((y1 - right_const) / right_slope);
  int right_x2 = int((y2 - right_const) / right_slope);
  int left_x1 = int((y1 - left_const) / left_slope);
  int left_x2 = int((y2 - left_const) / left_slope);
  int mid_x1 = (right_x1 + left_x1) / 2;
  int mid_x2 = (right_x2+ left_x2) / 2;

  if(draw_right){
     line(cameraFeed, Point(right_x1, y1), Point(right_x2, y2), BLUE, THICKNESS, CV_AA);
  }

  if(draw_left){
     line(cameraFeed, Point(left_x1, y1), Point(left_x2, y2), BLUE, THICKNESS, CV_AA);
  }
  if(draw_right && draw_left){
     line(cameraFeed, Point(mid_x1, y1), Point(mid_x2, y2), BLACK, THICKNESS, CV_AA);
     line(cameraFeed, Point(320,0), Point(320,640), BLACK, 0, CV_AA);
     //line(cameraFeed, Point(102,0), Point(102,640), BLACK, 0, CV_AA);
     //line(cameraFeed, Point(577,0), Point(577,640), BLACK, 0, CV_AA);
     error = (mid_x2 - 320)/PIX_PER_CM;
     cout << error << "\n";
  } else {
     error = 9999;
  }
   

}



/* Adds some filters to the original image */
void LaneDetection::setROI(Mat img_full, Mat & res){
    Mat mask = Mat::zeros(img_full.size(), img_full.type());
    Point roi_corners[][6] = {{Point(0,640),Point(0,350),Point(320,300),Point(320,300),Point(640,350),Point(640,640)}};
    const Point * pp[1] = {roi_corners[0]};
    int numOfPoints[] = {6}; 
    fillPoly(mask, pp, numOfPoints, 1, BLACK);
    Mat re;
    bitwise_and(img_full, mask, re);
    res = re;
}

void LaneDetection::openCap() {
    cap.open(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
}


void LaneDetection::detectColor(Mat cameraFeed, Mat &result, Scalar lowerBoundColor, Scalar upperBoundColor, int pixelThresh){
   Mat res, hsvImage;
      cvtColor(cameraFeed, hsvImage, COLOR_BGR2HSV);
      Mat mask = Mat::zeros(cameraFeed.size(), cameraFeed.type());


      inRange(hsvImage, lowerBoundColor, upperBoundColor, mask);
      bitwise_and(cameraFeed, cameraFeed, res, mask);
      int nonZeroCount = countNonZero(mask);
      String color = pixelThresh == 9000 ? "YELLOW SIGN" : "GREEN SIGN";
      putText(cameraFeed, color + ": ", Point(10, 50), font, 1, Scalar(0,0,255), 8);
      putText(cameraFeed,  "Pixel " + to_string(nonZeroCount), Point(20,80), font, 1, Scalar(255,0,0), 1);


      if(nonZeroCount > pixelThresh){
         putText(cameraFeed,"Truck at turn! ", Point(100,200), font, 2, Scalar(0,0,255), 8);
      }

      result = cameraFeed;

}

void LaneDetection::closeCap() {
    cap.release();
}

double LaneDetection::runLD(Mat frame){
       Mat cameraFeed, ipm, histImage, edge, roi_img, grayScale, binary_img, resized;
       vector<Vec4i> lines;

       DistanceMeasurement dm;
       
       VideoCapture cap;
       cap.open("output.avi");
       cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
       cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
       
       namedWindow("Window", 1);
       resizeWindow("Window", 640,480);
       while(cap.isOpened()) {
         cap.read(cameraFeed);
         setROI(cameraFeed, roi_img);
         cvtColor(ipm, grayScale,  COLOR_BGR2GRAY);
         equalizeHist(grayScale, histImage);
         threshold(histImage, binary_img, 235, 255, 0);
         Canny(binary_img, edge, 200, 255, 3);
         HoughLinesP(edge, lines, 1, M_PI/180, 50, 10, 10);
         drawLanes(cameraFeed, lines, 0.3, 0.4);
         usleep(70000);
         imshow("Window", cameraFeed);
          switch(waitKey(1)){
             case 27: // Press ESC to exit
               cameraFeed.release();
    	       destroyAllWindows();
               cap.release();
               exit(0);
          }
      }

}


int main(int argc, char *argv[])
{
    LaneDetection ld;
    Mat mat;
    ld.runLD(mat);    
}
