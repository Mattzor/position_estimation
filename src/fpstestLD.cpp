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







#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include<unistd.h>
#include <iostream>
#include "utils/headers/posest.h"
#include <sys/time.h>
#include <ctime>
#include <thread>
#include <queue>
#include <condition_variable>
#include <mutex>
using namespace std;
using namespace cv;
using Clock = std::chrono::steady_clock;
using std::chrono::time_point;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
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
#define WIDTH 360 
#define HEIGHT 480

/* linear regression function */
void linearFit(double &slope, double &m, vector<int>& xValues, vector<int>& yValues)
{
    int i,n;
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
void findSlopes(vector<Vec4i> lines, double thresholdSlope, vector<float>& s, vector<Vec4i>& tmp){
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


void estimateOptimalLane(vector<Vec4i> lines, double& slope, double& lineConst, bool& drawLine){
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



double laneerror;
/* Function for drawing the lines on the screen */
void drawLanes(Mat cameraFeed, vector<Vec4i> lines, double thresholdSlope, double trapH){
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
     //line(cameraFeed, Point(right_x1, y1), Point(right_x2, y2), BLUE, THICKNESS, CV_AA);
  }

  if(draw_left){
      //line(cameraFeed, Point(left_x1, y1), Point(left_x2, y2), BLUE, THICKNESS, CV_AA);
  }
  if(draw_right && draw_left){
     //line(cameraFeed, Point(mid_x1, y1), Point(mid_x2, y2), BLACK, THICKNESS, CV_AA);
     laneerror = mid_x1 - 320;
  } else {
    laneerror = 9999;
  }
   

}



/* Adds some filters to the original image */
void setROI(Mat img_full, Mat & res){
    Mat mask = Mat::zeros(img_full.size(), img_full.type());
    Point roi_corners[][6] = {{Point(0,360),Point(0,200),Point(240,150),Point(240,150),Point(480,200),Point(480,360)}};
    const Point * pp[1] = {roi_corners[0]};
    int numOfPoints[] = {6}; 
    fillPoly(mask, pp, numOfPoints, 1, BLACK);
    Mat re;
    bitwise_and(img_full, mask, re);
    res = re;
}


void setPosEstROI(Mat img_full, Mat & res){
    Point roi_corners[][6] = {{Point(0,360),Point(0,200),Point(240,150),Point(240,150),Point(480,200),Point(480,360)}};
    Point * pp[1] = {roi_corners[0]};
    posest::setROI(pp, 6 ,img_full, res);
}

void setPosEstROIColorGray(Mat img_full, Mat & res){
    Point roi_corners[][6] = {{Point(0,360),Point(0,200),Point(240,150),Point(240,150),Point(480,200),Point(480,360)}};
    Point * pp[1] = {roi_corners[0]};
    posest::setROIColorImageGray(pp, 6 ,img_full, res);
}

bool shouldwerun;
mutex mtx1,mtx2,mtx3,mtx4,mtx5;
int iterations;
void applyROI(queue<Mat> *out, VideoCapture *cap, condition_variable *consumer) {
    bool run = true;
    int iterations = 0;
    while(run) {
        Mat img, src;
        (*cap).read(img); 
        resize(img, src, Size(480, 360));
        Mat roi;
        setROI(src, roi);
        (*out).push(roi);
        (*consumer).notify_one();
        iterations++;
        if(iterations == 200) {
          run = false;
        }
    }
}

/*
    Some of the following methods have this same format.
    A simple producer-consumer chain. A input-queue and a output-queue is given, one that is read from and one that is written from.
    A condition_variable for the producer and the consumer is given. This method/thread will sleep untill the producer thread has signalled it to start.
    When one iteration of the code has executed, another condition_variable is told to wake up the next thread in the chain, so that it may continue.
*/
void applyGrayScale(queue<Mat> *in, queue<Mat> *out, condition_variable *producer, condition_variable *consumer) {
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx1);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        Mat img = (*in).front(); (*in).pop();
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        (*out).push(gray);
        (*consumer).notify_one();
    }
}

void applyHist(queue<Mat> *in, queue<Mat> *out, condition_variable *producer, condition_variable *consumer) {
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx2);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        Mat img = (*in).front(); (*in).pop();
        Mat hist;
        equalizeHist(img, hist);
        (*out).push(hist);
        (*consumer).notify_one();
        iterations++;
    }
}

void applyBin(queue<Mat> *in, queue<Mat> *out, condition_variable *producer, condition_variable *consumer) {
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx3);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        Mat img = (*in).front(); (*in).pop();
        Mat bin;
        threshold(img, bin, 242, 255, 0);
        (*out).push(bin);
        (*consumer).notify_one();
        iterations++;
    }
}

void applyCanny(queue<Mat> *in, queue<Mat> *out, condition_variable *producer, condition_variable *consumer) {
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx4);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        Mat img = (*in).front(); (*in).pop();
        Mat can;
        Canny(img, can, 200, 255, 3);
        (*out).push(can);
        (*consumer).notify_one();
        iterations++;
    }
}

void applyHughLines(queue<Mat> *in, condition_variable *producer, condition_variable *consumer) {
    int iterations = 0;
    time_t start;
    time(&start);
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx5);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        vector<Vec4i> lines;
        Mat img = (*in).front(); (*in).pop();
        HoughLinesP(img, lines, 1, M_PI/180, 50, 10, 10);
        drawLanes(img, lines, 0.3, 0.4);
        iterations++;
        (*consumer).notify_one();
        time_t now;
        time(&now);
        if(difftime(now,start) == 10) {
          cout << ((double)iterations/10) << endl;
          shouldwerun = false;
        }
    }
}

/*
  Applies our own setROI, one that actually crops the image and not just reduces noise.
*/
void applyBetterROI(queue<Mat> *in, queue<Mat> *out, condition_variable *producer, condition_variable *consumer) {
    while(shouldwerun) {
        unique_lock<mutex> lck(mtx1);
        while((*in).size() == 0) {
          (*producer).wait(lck);
        }
        Mat img = (*in).front(); (*in).pop();
        Mat roi;
        setPosEstROI(img, roi);
        (*out).push(roi);
        (*consumer).notify_one();
        iterations++;
    }
}

/*
  Applies our own setROI, one that actually crops the image and not just reduces noise. This one also grayscales.
*/
void applyROIGrayScale(queue<Mat> *out, VideoCapture *cap, condition_variable *consumer) {
    while(shouldwerun) {
        Mat img, src;
        (*cap).read(img); 
        resize(img, src, Size(480, 360));
        Mat roi;
        setPosEstROIColorGray(src, roi);
        (*out).push(roi);
        (*consumer).notify_one();
        iterations++;
    }
}

/*
  Extracts a picture and grayscales it.
*/
void applyGrayScaleAndExtract(queue<Mat> *out, VideoCapture *cap, condition_variable *consumer) {
    while(shouldwerun) {
        Mat img;
        (*cap).read(img);
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        (*out).push(gray);
        (*consumer).notify_one();
        iterations++;
    }
}
int main(int argc, char *argv[])
{
    shouldwerun = true;
    Mat cameraFeed, src, histImage, edge, roi_img, grayScale, binary_img;
    VideoCapture cap;
    cap.open(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

    queue<Mat> roi, gray, hist, bin, canny;

    condition_variable inroi, ingray, inhist, inbin, incanny, inlines, endconsumer;
    iterations = 0;

    /*
      Same as the above one, but it also grayscales as it sets ROI.
    */
    iterations = 0; 
    thread firstB2 (applyROIGrayScale, &roi, &cap, &ingray);
    thread thirdB2 (applyHist, &roi, &hist, &ingray, &inbin);
    thread fourthB2 (applyBin, &hist, &bin, &inbin, &incanny);
    thread fifthB2 (applyCanny, &bin, &canny, &incanny, &inlines);
    thread sixthB2 (applyHughLines, &canny, &inlines, &endconsumer);
    firstB2.join();
    thirdB2.join();
    fourthB2.join();
    fifthB2.join();
    sixthB2.join();

    /*
      Close streams and exit.
    */
    cap.release();
    exit(1);
}









