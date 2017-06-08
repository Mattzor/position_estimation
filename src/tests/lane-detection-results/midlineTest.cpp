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
#include "../../laneDetection.h"
#include <stdlib.h>
#include <stdio.h>
#include<unistd.h>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace cv;
using namespace std;

#define x1 1161
#define x2 1163
#define y1 2399
#define y2 7405

void parseGulliview(string line, int *x, int *y);
void whatSideOfLine(int gulx, int guly, bool *leftside);
void distanceToPoint(int gulx, int guly, double *distance);

ifstream gulliview;
VideoCapture cap;
LaneDetection ld;


/*
  Evaluate LaneDetection. Use positions from gulliview to get the vehicles actual position.
  Check on what side of the midline the vehicle is. Compare what side the vehicle is at to
  which side lanedetection says the vehicle is at.

  The midline goes from the point (1161,2399) to (1163,7405).
*/
int main(int argc, char **argv) {
  /* Declare locals */
  string line;
  Mat frame;
  double laneerror;
  int nrframes = 0;
  int faultyframes = 0; // Number of frames where lanedetection gave a false value.
  int gulx,guly;
  double x,slope,intercept;
  double distance;
  bool leftside; // true = left and false if it is on or to the right of the line.

  /* Open streams */
  gulliview.open("gulliview.txt");
  cap.open("output.avi");

  /* Meat of the test */
  while(getline(gulliview,line)) {
    if(cap.isOpened()) {
      /* Read a frame and process it to find out what side lane detection thinks we're on. */
      cap.read(frame);
      laneerror = ld.runLD(frame);
      waitKey(0);

      /* Parse the gulliviewposition and check what side the position is of the midline. */
      parseGulliview(line, &gulx, &guly);
      whatSideOfLine(gulx, guly, &leftside);
      if(laneerror != 9999 && abs(laneerror) < 250) {
      /*if(guly >= y1 && guly <= y2 && (laneerror != 9999) && ((leftside && (laneerror > 0)) || (!leftside && (laneerror <= 0)))) {
        nrframes++;
      } else {
        nrframes++;
        faultyframes++;
      }*/
        nrframes++;
      } else {
        faultyframes++;
        nrframes++;
      }
    } else {
      cout << "Error: cap not opened" << endl;
      return 0;
    }
  }
  cout << "Faulty reading from " << faultyframes << " frames out of " << nrframes << " frames. The ratio of good processings is = " << (((double)nrframes - (double)faultyframes)/(double)nrframes) << "." << endl;
  return 1;
}

/*
  Parses a line of text representing a gulliview message into two ints, x and y. These are then put in the supplied integer pointers.
*/
void parseGulliview(string line, int *x, int *y) {
  int gx, gy;
  int index;

  index = line.find(' ');
  gx = stoi(line.substr(0,index));
  gy = stoi(line.substr(index+1,line.length()-index));
  *x = gx; *y = gy;
}

/*
  Checks if a given point is to the left or right of the given line.
*/
void whatSideOfLine(int gulx, int guly, bool *leftside) {
  double reference = (x1-1-x1)*(y2-y1)-(y1-y1)*(x2-x1);
  double pointsign = (gulx-x1)*(y2-y1)-(guly-y1)*(x1-x2);
  
  *leftside = ((reference < 0) ? ( pointsign < 0) : ((reference > 0) ? (pointsign > 0) : (pointsign == 0)));
}

/*
  Calculates and return the distance from the point (gulx,guly) to the line given by x1,x2,y1,y2.
  The result is placed in *distance.
*/
void distanceToPoint(int gulx, int guly, double *distance) {
  double numerator   = abs((y2-y1)*gulx - (x2-x1)*guly + x2*y1 - y2*x1);
  double denominator = sqrt(pow(y2-y1,2) + pow(x2-x1,2));
  *distance = numerator/denominator;
}
