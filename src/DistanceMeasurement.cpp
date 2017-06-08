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
#include<math.h>
using namespace cv;
using namespace std;




// perspevtive transform function
void DistanceMeasurement::pTransform(Mat frame, Mat &result){
   Mat transformedImage;
   int width = frame.size().width;
   int height = frame.size().height;
   Point2f pointsA[4] = {Point2f(0, height), 
                         Point2f(0, scale * scale_frame.size().height), 
                         Point2f(width, scale * scale_frame.size().height), 
                         Point2f(width, height)};

   
   Point2f pointsB[4] = {Point2f(50.0 / 128 * width, height), 
                          Point2f(0 , 0), 
                          Point2f(width, 0), 
                          Point2f(78.0 / 128 * width, height)};
   transformedImage = getPerspectiveTransform(pointsA, pointsB);
   warpPerspective(frame, result, transformedImage, result.size());
}



// opens videocapture and sets width, height
void DistanceMeasurement::openCap(){
   cap.open(0); // change to 0 later
   cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
}



// releases the capture 
void DistanceMeasurement::closeCap(){
   cap.release();
}



// returns the length between two points in 
float DistanceMeasurement::lengthOfPoints(Point a , Point b){
   return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}




// returns true if the difference of the three sides is smaller than triangle_length
bool DistanceMeasurement::sizeOfTriangle(Point a, Point b, Point c){
	int length_1 = lengthOfPoints(a,b);
	int length_2 = lengthOfPoints(b,c);
	int length_3 = lengthOfPoints(c,a);
        bool allSidesOk = (abs(length_1 - length_2) < triangleSizeDiff) && 
		          (abs(length_2 - length_3) < triangleSizeDiff) && 
		          (abs(length_3 - length_1) < triangleSizeDiff);
	
        if(allSidesOk){
                bool vectorsOk = (length_1 >= standardLength) && 
                                 (length_2 >= standardLength) && 
                                 (length_3 >= standardLength);
		if(vectorsOk){
			return true;
		}
	}
	return false;
}



// checks if the polygon is a tringale and also if the triangle has the right dimensions 
bool DistanceMeasurement::triangleOk(vector<Point> points){
	if(points.size() == edges && sizeOfTriangle(points[0], points[1], points[2]) ){
		return true;
	}
	return false;
}




void DistanceMeasurement::linePointSettings(){
   // Point to move the visual lines. 
   move.x = startX;
   move.y = startY;
   // center down point of the image.
   double width =  cap.get(CV_CAP_PROP_FRAME_WIDTH);
   double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   startPoint.x = width/2;
   startPoint.y = height;
   
}



void DistanceMeasurement::setContours(Mat clonedImage, Mat &cannyOut, vector<vector<Point> > &contours){
   Rect rect;
   scale_frame = clonedImage.clone();
   rect.x = startX, rect.y = startY;
   rect.width = roiWidth, rect.height = roiHeight;
   clonedImage = clonedImage(rect);
   cvtColor(clonedImage, clonedImage,COLOR_BGR2GRAY);
   Canny(clonedImage, cannyOut, 100, 200, 3);
   vector<Vec4i> hierarchy;
   findContours( cannyOut, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
}



void DistanceMeasurement::drawLines(Mat &cameraFeed, vector<vector<Point> > contours){
   Point temp;
   temp.x = startX;
   temp.y = startY;

   Point temp_2;
   temp_2.x = 34;
   temp_2.y = HEIGHT;
   

   //line(cameraFeed, move, , Scalar(0,255,0), 5,8,0);
   for(int i = 0; i < contours.size(); i++){
      vector<Point> result;
      approxPolyDP(contours[i], result, 10, true);
      if(triangleOk(result)){
         // here we have found a sign. 
        // this+ is just for drawing.
        setLenOfTri(result);
	for(int j = 0; j < edges; j++){
           if(j+1 <  edges){
              Point m = move;
              Point s = startPoint;

   	      line(cameraFeed,result[j] + move, result[j+1] + move, Scalar(0,255,0), 5,8,0);
           }else{
              line(cameraFeed,result[j] + move, result[0]+ move, Scalar(0,255,0), 5,8,0);
   	   }
           line(cameraFeed,result[j] + move , startPoint, Scalar(255,0,0), 2,8,0);	
        }
	
     }
   }
}
  

bool DistanceMeasurement::setLenOfTri(vector<Point> sidesOfTri){
   Point a,b,c;
   Point p1 = sidesOfTri[0] + move;
   Point p2 = sidesOfTri[1] + move;
   Point p3 = sidesOfTri[2] + move;
   float p1p2 = lengthOfPoints(p1, p2);
   float p1p3 = lengthOfPoints(p1, p3);
   float p2p3 = lengthOfPoints(p2, p3);
   if(p1p2 > p1p3 && p1p2 > p2p3){
     a = p1;
     b = p2;
     c = p3;
   }else if (p1p3 > p1p2 && p1p3 > p2p3){
     a = p1;
     b = p3;
     c = p2;
   }else{
     a = p2;
     b = p3;
     c = p1;
   }

   calculateDis(a, b, c);
}


void DistanceMeasurement::calculateDis(Point a, Point b, Point c){
   Point midOfAB, midOfTri;
   midOfAB.x = a.x + (b.x - a.x)/2;
   midOfAB.y = a.y + (b.y - a.y)/2;
   midOfTri.x = c.x + (midOfAB.x - c.x)/2;
   midOfTri.y = c.y + (midOfAB.y - c.y)/2;
   truckPos.x = WIDTH/2;   // 32 pixels between the position of the truck and the camera
   truckPos.y = HEIGHT + 150;
   focalLength = lengthOfPoints(truckPos, midOfTri)*SCALE;
   truckAngle = rotationAngle(c, midOfAB);
   globalAngle = truckSignAngle(midOfTri, truckPos, c, midOfAB, truckAngle);
}



float DistanceMeasurement::truckSignAngle(Point midOfTri, Point truckPos, Point c, Point midOfAB, float angle){
   float x = midOfTri.x - truckPos.x;
   float y = midOfTri.y - truckPos.y;
   float theta;
   float alpha = atan(y/x) * 180 / 3.14159265;
   theta = 90 - alpha;
   if(theta > 90){
      theta -= 180;
   }
   /*Point w1, w2;
   if(midOfTri.x < truckPos.x){
      theta = 90 - (180 - (angle + alpha));
	if(c.y >= midOfTri.y  && c.x > midOfTri.x)
	{
          theta -=90;
	}else{
           if(alpha > 80){
              theta -= 90;
           }
        }
   }else if(midOfTri.x > truckPos.x){
      alpha *= -1;
      theta =90 + (180 - (angle + alpha));
   }
   if(theta < 0)
      theta += 360;*/
   return theta;
}


float DistanceMeasurement::rotationAngle(Point c, Point midOfAB){
   float x = c.x - midOfAB.x; 
   if(x == 0){
      x = 1;
   }
   float cofficient = (c.y - midOfAB.y) / (x);
   float angle = atan(cofficient) * 180 / 3.14159265;
   /*if (angle < 0)
      angle = 180 + angle;*/
   float newAngle = 0;
   if(c.x > midOfAB.x && c.y > midOfAB.y){
      newAngle += 180 + angle; 
   }else if(c.x >= midOfAB.x && c.y < midOfAB.y){
      newAngle += 180 + angle;
   }else if(c.x < midOfAB.x && c.y < midOfAB.y){
      newAngle = angle;   
   }else if(c.x < midOfAB.x && c.y > midOfAB.y){
      newAngle += 360 + angle;
   }else{
      //cout << "other cases\n";
   }

   if(newAngle == 0)
	newAngle = 270;
   return newAngle;
}




