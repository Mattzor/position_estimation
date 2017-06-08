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
#include "../../headers/posest.h"
#include "../utils/assert.h"
using namespace std;
using namespace posest;

bool testSmallSetROI() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};

    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return cv::countNonZero(res) > 0;
}

bool testSetROIDim() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,19),cv::Point(10,18),cv::Point(10,18),cv::Point(20,19),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};

    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return cv::countNonZero(res) > 0 && res.rows == 2 && res.cols == 20;
}

bool testActualValues() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,10),cv::Point(10,8),cv::Point(10,8),cv::Point(20,10),cv::Point(20,20)}};
    cv::Point *pp[1] = {roi_corners[0]};   
 
    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return (12 * 20 - cv::countNonZero(res)) == 29;
}

void testSetROITests() {
  cout << ((testSmallSetROI())    ? "testSmallSetROI  holds" : "testSmallSetROI  fails") << endl;
  cout << ((testSetROIDim())      ? "testSetROIDim    holds" : "testSetROIDim    fails") << endl;
  cout << ((testActualValues())   ? "testActualValues holds" : "testActualValues fails") << endl;
}
