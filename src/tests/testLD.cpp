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
#include "../laneDetection.h"
#include<unistd.h>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"

using namespace std;
using namespace cv;

bool assertMatEquals(const Mat *one, const Mat *two);
bool assertDoubleEquals(const double one, const double two);
bool assertVec4iEquals(const Vec4i one, const Vec4i two);

/* tests on setROI */

/*
  A simple property test that checks if setROI actually produces a different image.
*/
bool testROISimple() {
  double *ptr1 = new double[360*480];
  double *ptr2 = new double[360*480];
  Mat img(Mat(360, 480, CV_8U, ptr1));
  Mat res(Mat(360, 480, CV_8U, ptr2));
  img = Scalar(255);

  LaneDetection ld;
  ld.setROI(img, res);
  bool assertion = assertMatEquals(&img, &res);
  delete[] ptr1;
  delete[] ptr2;
  return !assertion;
}

/*
  Tests that there is no dependency on state or anything. Given the same image twice, setROI should return the exact same image.
*/
bool testROISameResultTwice() {
  double *ptr1 = new double[360*480];
  double *ptr2 = new double[360*480];
  double *ptr3 = new double[360*480];
  Mat one(Mat(360, 480, CV_8U, ptr1));
  one = Scalar(255);

  Mat res1(Mat(360, 480, CV_8U, ptr2));
  Mat res2(Mat(360, 480, CV_8U, ptr3));

  LaneDetection ld;
  ld.setROI(one, res1);
  ld.setROI(one, res2);
  bool assertion = assertMatEquals(&res1, &res2);
  delete[] ptr1;
  delete[] ptr2;
  delete[] ptr3;
  return assertion;
}

/*
  simple test for assertMatEquals, to make sure that it works.
*/
bool testMatEqualsSimple() {
  double *ptr1 = new double[360*480];
  double *ptr2 = new double[360*480];
  Mat img(Mat(360, 480, CV_8U, ptr1));
  Mat res(Mat(360, 480, CV_8U, ptr2));
  img = Scalar(255);
  res = Scalar(255);

  bool assertion = assertMatEquals(&img, &res);
  delete[] ptr1;
  delete[] ptr2;
  return assertion;
}

/*
  simple test for assertMatEquals, to make sure that it works.
*/
int testMatEqualsWrongSizes() {
  double *ptr1 = new double[360*480];
  double *ptr2 = new double[360*280];
  Mat one(Mat(360, 480, CV_8U, ptr1));
  Mat two(Mat(360, 280, CV_8U, ptr2));
  bool assertion = assertMatEquals(&one, &two);
  delete[] ptr1;
  delete[] ptr2;
  return !assertion;
}

/*
  Makes sure that setROI does not alter the original Mat. If the same image can be re-used we can save computing time.
*/
bool testSetROIUnchangedOriginal() {
  double *ptr1 = new double[360*480];
  double *ptr2 = new double[360*480];
  double *ptr3 = new double[360*480];
  Mat one(Mat(360, 480, CV_8U, ptr1));
  Mat two(Mat(360, 480, CV_8U, ptr2));
  Mat three(Mat(360, 480, CV_8U, ptr2));
  
  one = Scalar(255);
  three = Scalar(255);
  bool isOriginallyEqual = assertMatEquals(&one, &three);  

  LaneDetection ld;
  ld.setROI(one, two);
  bool isStillEqual = assertMatEquals(&one, &three);
  bool twoIsChanged = !assertMatEquals(&one, &two);
  delete[] ptr1;
  delete[] ptr2;
  delete[] ptr3;
  return isOriginallyEqual && isStillEqual && twoIsChanged;
}
/* ------------------------------------------------------------ */
/* Tests on linear fit */

/*
  Simple test. The best fit is a y = x;
*/
bool testLinearFitSimple() {
  double expectedSlope, expectedIntercept, calculatedSlope, calculatedIntercept;
  vector<int> xs, ys;
  xs.push_back(5);
  xs.push_back(7);
  ys.push_back(5);
  ys.push_back(7);
  
  expectedSlope = 1.0;
  expectedIntercept = 0.0;

  LaneDetection ld;
  ld.linearFit(calculatedSlope, calculatedIntercept, xs, ys);
  bool slopeEquals = assertDoubleEquals(expectedSlope, calculatedSlope);
  bool interceptEquals = assertDoubleEquals(expectedIntercept, calculatedIntercept);
  return slopeEquals && interceptEquals;
}

/*
  Simple test. The best fit should be y = x + 0.33~;
*/
bool testLinearFitThreePoints() {
  double expectedSlope, expectedIntercept, calculatedSlope, calculatedIntercept;
  vector<int> xs, ys;
  xs.push_back(5);
  xs.push_back(7);
  xs.push_back(9);
  ys.push_back(5);
  ys.push_back(8);
  ys.push_back(9);

  expectedSlope = 1.0;
  expectedIntercept = 0.3333333333333333333333;
  
  LaneDetection ld;
  ld.linearFit(calculatedSlope, calculatedIntercept, xs, ys);
  bool slopeEquals = assertDoubleEquals(expectedSlope, calculatedSlope);
  bool interceptEquals = assertDoubleEquals(expectedIntercept, calculatedIntercept);
  return slopeEquals && interceptEquals;
}

/*
  Another test where the expected line is fairly simple, y=0.8x + 1.5;
*/
bool testLinearFitFourPoints() {
  double expectedSlope, expectedIntercept, calculatedSlope, calculatedIntercept;
  vector<int> xs, ys;
  xs.push_back(6);
  xs.push_back(7);
  xs.push_back(8);
  xs.push_back(9);
  ys.push_back(6);
  ys.push_back(8);
  ys.push_back(7);
  ys.push_back(9);

  expectedSlope = 0.8;
  expectedIntercept = 1.5;

  LaneDetection ld;
  ld.linearFit(calculatedSlope, calculatedIntercept, xs, ys);
  bool slopeEquals = assertDoubleEquals(expectedSlope, calculatedSlope);
  bool interceptEquals = assertDoubleEquals(expectedIntercept, calculatedIntercept);
  return slopeEquals && interceptEquals;
}

/*
  A slightly bigger test with 8 datapoints. The best fit with linear regression is y = 0.41666666x + 10.416666666;
*/
bool testLinearFitManyPoints() {
  double expectedSlope, expectedIntercept, calculatedSlope, calculatedIntercept;
  vector<int> xs, ys;
  xs.push_back(6);
  xs.push_back(7);
  xs.push_back(8);
  xs.push_back(9);
  xs.push_back(10);
  xs.push_back(11);
  xs.push_back(12);
  xs.push_back(13);

  ys.push_back(16);
  ys.push_back(12);
  ys.push_back(14);
  ys.push_back(10);
  ys.push_back(13);
  ys.push_back(17);
  ys.push_back(18);
  ys.push_back(15);
  
  expectedSlope = 0.41666666666666;
  expectedIntercept = 10.41666666666;

  LaneDetection ld;
  ld.linearFit(calculatedSlope, calculatedIntercept, xs, ys);
  bool slopeEquals = assertDoubleEquals(expectedSlope, calculatedSlope);
  bool interceptEquals = assertDoubleEquals(expectedIntercept, calculatedIntercept);
  return slopeEquals && interceptEquals;
}
/* ------------------------------------------------------------ */
/* Tests on findSlopes */


/*
  A test that makes sure the right lines are included in the resulting vector.
*/
bool testSimpleFindSlopes() {
  vector<Vec4i> inputLines, expectedOutputLines, calculatedLines;
  vector<float> expectedSlopes, calculatedSlopes;

  inputLines.push_back(Vec4i(1,1,2,2));
  inputLines.push_back(Vec4i(2,2,4,5));
  inputLines.push_back(Vec4i(3,3,40,4));
  inputLines.push_back(Vec4i(4,4,50,-5));

  expectedSlopes.push_back(1);
  expectedSlopes.push_back(1.5);

  expectedOutputLines.push_back(Vec4i(1,1,2,2));
  expectedOutputLines.push_back(Vec4i(2,2,4,5));

  LaneDetection ld;
  ld.findSlopes(inputLines, 0.5, calculatedSlopes, calculatedLines);

  bool slopesEqual = 1, linesEqual = 1;
  for(int i = 0; i < calculatedLines.size(); i++) {
    if(!assertVec4iEquals(calculatedLines[i], expectedOutputLines[i])) {
        linesEqual = false;
    }
    if(!assertDoubleEquals(expectedSlopes[i], calculatedSlopes[i])) {
        slopesEqual = false;
    }
  }
  return slopesEqual && linesEqual;
}

bool testFindSlopesEmptyResult() {
  vector<Vec4i> inputLines, expectedOutputLines, calculatedLines;
  vector<float> expectedSlopes, calculatedSlopes;

  inputLines.push_back(Vec4i(3,3,40,4));
  inputLines.push_back(Vec4i(4,4,50,-5));

  LaneDetection ld;
  ld.findSlopes(inputLines, 0.5, calculatedSlopes, calculatedLines);

  bool slopesEqual = 1, linesEqual = 1;
  for(int i = 0; i < calculatedLines.size(); i++) {
    if(!assertVec4iEquals(calculatedLines[i], expectedOutputLines[i])) {
        linesEqual = false;
    }
    if(!assertDoubleEquals(expectedSlopes[i], calculatedSlopes[i])) {
        slopesEqual = false;
    }
  }
  return slopesEqual && linesEqual;
}

bool testFindSlopesEverythingIncluded() {
  vector<Vec4i> inputLines, expectedOutputLines, calculatedLines;
  vector<float> expectedSlopes, calculatedSlopes;

  inputLines.push_back(Vec4i(1,1,2,2));
  inputLines.push_back(Vec4i(2,2,4,5));

  expectedSlopes.push_back(1);
  expectedSlopes.push_back(1.5);

  expectedOutputLines.push_back(Vec4i(1,1,2,2));
  expectedOutputLines.push_back(Vec4i(2,2,4,5));

  LaneDetection ld;
  ld.findSlopes(inputLines, 0.5, calculatedSlopes, calculatedLines);

  bool slopesEqual = 1, linesEqual = 1;
  for(int i = 0; i < calculatedLines.size(); i++) {
    if(!assertVec4iEquals(calculatedLines[i], expectedOutputLines[i])) {
        linesEqual = false;
    }
    if(!assertDoubleEquals(expectedSlopes[i], calculatedSlopes[i])) {
        slopesEqual = false;
    }
  }
  return slopesEqual && linesEqual;
}
/* ------------------------------------------------------------ */
/* Tests for estimateOptimalLane */
bool testGoodOptimal() {
  vector<Vec4i> inputLines;
  inputLines.push_back(Vec4i(1,1,2,2));
  inputLines.push_back(Vec4i(2,2,3,3));
  inputLines.push_back(Vec4i(3,3,4,4));

  double slope, intercept;
  bool draw;

  double expectedSlope = 1.0, expectedIntercept = 0;
  bool expectedDraw = true;

  LaneDetection ld;
  ld.estimateOptimalLane(inputLines, slope, intercept, draw);

  return assertDoubleEquals(expectedSlope, slope) && assertDoubleEquals(expectedIntercept, intercept) && (draw == expectedDraw);
}

bool testEmptyOptimal() {
  vector<Vec4i> inputLines;

  double slope, intercept;
  bool draw;

  double expectedSlope = 1.0, expectedIntercept = 1.0;
  bool expectedDraw = false;

  LaneDetection ld;
  ld.estimateOptimalLane(inputLines, slope, intercept, draw);

  return assertDoubleEquals(expectedSlope, slope) && assertDoubleEquals(expectedIntercept, intercept) && (draw == expectedDraw);
}
/* ------------------------------------------------------------ */
/*
  Performs a bitwise XOR on two matrices, and if the result is all 0's, they're equal.
*/
bool assertMatEquals(const Mat *one, const Mat *two) {
    if(one->rows == two->rows && one->cols == two->cols) {
      Mat temp;
      bitwise_xor(*one, *two, temp);
      return !(cv::countNonZero(temp));
    }
    return false;
}

/*
  given two doubles, returns diff < 0.000001
*/
bool assertDoubleEquals(const double one, const double two) {
    double diff = ((one - two) < 0) ? -(one - two) : (one - two); /* simple abs(one-two) */
    return (diff < 0.000001);
}

/*
  Given two vectors, checks wether all elements are equal.
*/
bool assertVec4iEquals(const Vec4i one, const Vec4i two) {
    return (assertDoubleEquals(one[0], two[0]) && assertDoubleEquals(one[1], two[1]) && assertDoubleEquals(one[2], two[2]) && assertDoubleEquals(one[3], two[3]));
}

/*
  Enter all tests here.
*/
int main(int argc, char *argv[]) {
  cout << (testROISimple() ?                    "testROISimple()                    assertion holds\n": "testROISimple()                    assertion failed\n" );
  cout << (testMatEqualsSimple() ?              "testMatEqualsSimple()              assertion holds\n": "testMatEqualsSimple()              assertion failed\n");
  cout << (testMatEqualsWrongSizes() ?          "testMatEqualsWrongSizes()          assertion holds\n": "testMatEqualsWrongSizes()          assertion failed\n");
  cout << (testROISameResultTwice() ?           "testROISameResultTwice()           assertion holds\n": "testROISameResultTwice()           assertion failed\n");
  cout << (testSetROIUnchangedOriginal() ?      "testSetROIUnchangedOriginal()      assertion holds\n": "testSetROIUnchangedOriginal()      assertion failed\n");
  cout << (testLinearFitSimple() ?              "testLinearFitSimple()              assertion holds\n": "testLinearFitSimple()              assertion failed\n");
  cout << (testLinearFitThreePoints() ?         "testLinearFitThreePoints()         assertion holds\n": "testLinearFitThreePoints()         assertion failed\n");
  cout << (testLinearFitFourPoints() ?          "testLinearFitFourPoints()          assertion holds\n": "testLinearFitFourPoints()          assertion failed\n");
  cout << (testLinearFitManyPoints() ?          "testLinearFitManyPoints()          assertion holds\n": "testLinearFitManyPoints()          assertion failed\n");
  cout << (testSimpleFindSlopes() ?             "testSimpleFindSlopes()             assertion holds\n": "testSimpleFindSlopes()             assertion failed\n");
  cout << (testFindSlopesEmptyResult() ?        "testFindSlopesEmptyResult()        assertion holds\n": "testFindSlopesEmptyResult()        assertion failed\n");
  cout << (testFindSlopesEverythingIncluded() ? "testFindSlopesEverythingIncluded() assertion holds\n": "testFindSlopesEverythingIncluded() assertion failed\n");
  cout << (testGoodOptimal() ?                  "testGoodOptimal()                  assertion holds\n": "testGoodOptimal()                  assertion failed\n");
  cout << (testEmptyOptimal() ?                 "testEmptyOptimal()                 assertion holds\n": "testEmptyOptimal()                 assertion failed\n");
}








