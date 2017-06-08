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
