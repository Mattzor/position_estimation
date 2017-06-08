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
#include <opencv/cv.h>
#include "DistanceMeasurement.h"
#include <iostream>
#include <unistd.h>
using namespace cv;
using namespace std;




// perspevtive transform function
void DistanceMeasurement::pTransform(Mat frame, float yTop, Mat &result){
   Mat transformedImage;
   int width = frame.size().width;
   int height = frame.size().height;
   Point2f pointsA[4] = {Point2f(0, width), 
                         Point2f(0, yTop), 
                         Point2f(height, yTop), 
                         Point2f(height, width)};


   Point2f pointsB[4] = {Point2f(50.0 / 128 * height, width), 
                          Point2f(0 , 0), 
                          Point2f(height, 0), 
                          Point2f(78.0 / 128 * height, width)};
   transformedImage = getPerspectiveTransform(pointsA, pointsB);
   warpPerspective(frame, result, transformedImage, result.size());
}




int main(int argc, char *argv[]){
   VideoCapture cap;
   DistanceMeasurement d;
   Mat cameraFeed, res;
   cap.open("/home/hawre/catkin_ws/src/position_estimation/src/yellow1.mp4");
   namedWindow("Webcam", 1);
   while(cap.isOpened()){
      cap.read(cameraFeed);
      d.pTransform(cameraFeed, 335.0 / 720 * cameraFeed.size().width, cameraFeed);
      imshow("Webcam", cameraFeed);

      switch(waitKey(1)){
         case 27:
            exit(0);
      }
   }
}
