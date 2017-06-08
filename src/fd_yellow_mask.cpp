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
#include "opencv/cv.h"
#include <unistd.h> 
#include <iostream>

using namespace cv;
using namespace std;


#define lowerColor Scalar(40,60,0)
#define upperColor Scalar(75,255,255)

int font = FONT_HERSHEY_SCRIPT_SIMPLEX;
int pixel_thresh = 9000;



void detectColor(Mat cameraFeed, Mat &result, Scalar lowerBoundColor, Scalar upperBoundColor){
      Mat res, hsvImage;
      cvtColor(cameraFeed, hsvImage, COLOR_BGR2HSV);
      Mat mask = Mat::zeros(cameraFeed.size(), cameraFeed.type());


      inRange(hsvImage, lowerBoundColor, upperBoundColor, mask);
      bitwise_and(cameraFeed, cameraFeed, res, mask);
      int nonZeroCount = countNonZero(mask);
    
      putText(cameraFeed, "Yellow pixel count: ", Point(10, 50), font, 1, Scalar(0,0,255), 8);
      putText(cameraFeed,  "Pixel " + to_string(nonZeroCount), Point(20,80), font, 1, Scalar(255,0,0), 1);


      if(nonZeroCount > pixel_thresh){
         putText(cameraFeed,"Truck at turn! ", Point(100,200), font, 2, Scalar(0,0,255), 8);
      }

      result = cameraFeed;

}


int main(int argc, char *argv[]){
   VideoCapture cap;
   Mat cameraFeed, res;
   cap.open(0);
   namedWindow("Webcam", 1);
   while(cap.isOpened()){
      cap.read(cameraFeed);
      detectColor(cameraFeed, res, Scalar(10,120,120), Scalar(40,255,255));
      /*cvtColor(cameraFeed, hsvImage, COLOR_BGR2HSV);
      Mat mask = Mat::zeros(cameraFeed.size(), cameraFeed.type());
      Mat res;

      inRange(hsvImage, Scalar(20,120,120), Scalar(40,255,255), mask);
      bitwise_and(cameraFeed, cameraFeed, res, mask);
      int nonZeroCount = countNonZero(mask);
    
      putText(cameraFeed, "Yellow pixel count: ", Point(10, 50), font, 1, Scalar(0,0,255), 8);
      putText(cameraFeed,  "Pixel " + to_string(nonZeroCount), Point(20,80), font, 1, Scalar(255,0,0), 1);


      if(nonZeroCount > pixel_thresh){
         putText(cameraFeed,"Truck at turn! ", Point(100,200), font, 2, Scalar(0,0,255), 8);
      }*/
      //imshow("hsv_img", hsvImage);
      //imshow("mask", mask);
      //imshow("Filtered color only", res);
      imshow("Webcam", res);

      switch(waitKey(1)){
         case 27:
            exit(0);
      }
   }
}





