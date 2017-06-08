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
#include <omp.h>
using namespace std;

namespace posest {

    /*
      Point** corners expect 6 points.  {bottom left, middle left, top left, top right, middle right, bottom right}.
      Points should for a trapezoid shape.

      example of points, pp is what is given to the method: 
      cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
      cv::Point * pp[1] = {roi_corners[0]};
      
      Mat img is assumed to be a single channel (grayscaled) image.

      The resulting image has the width bottom.right.x iand height max(top.left.y, top.right.y).
    */
    void setROI(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res) {
      /* If anything other than 6 points have been supplied, reject */
      if(numpoint != 6) {
        cout << "Usage: supply 6 points to setROI(Point **corners, int numpoints, Mat & img, Mat & res). Anything else will be rejected." << endl;
        exit(0);
      } else {
        /* Calculate dimensions and initialize resulting cv::Mat */
        int x = (*(*corners + 5)).x;
        int y = img.rows - (((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y);
        cv::Mat result = cv::Mat::zeros(y, x, CV_8UC1);

        double firstSlope, firstIntersect, secondSlope, secondIntersect;
        firstSlope  = ((double)(*(*corners + 2)).y - (double)(*(*corners + 1)).y) / ((double)(*(*corners + 2)).x - (double)((*(*corners + 1)).x));
        secondSlope = ((double)(*(*corners + 2)).y - (double)(*(*corners + 4)).y) / ((double)(*(*corners + 2)).x - ((double)(*(*corners + 4)).x));
        firstIntersect  = (double)(*(*corners + 1)).y - (firstSlope * (double)(*(*corners + 1)).x);
        secondIntersect = (double)(*(*corners + 4)).y - (secondSlope * (double)(*(*corners + 4)).x);

        /* Calculate the different indexes to read and write from in the original cv::Mat */
        int ymin = ((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y;
        int ymax = img.rows;
        int ywrite = 0;
        #pragma omp parallel for
        for(int i = ymin; i < ymax; i++) {
            unsigned char* readrow = img.ptr<unsigned char>(i);
            unsigned char* writerow = result.ptr<unsigned char>(ywrite);
            
            /* If this row is in the 'top' segment of the ROI-polygon */
            if(i < (*(*corners + 1)).y) {
               int firstx = (i - firstIntersect) / firstSlope;
               int lastx = (i - secondIntersect) / secondSlope;
               if(firstx == lastx) {
                   writerow[firstx -1] = readrow[firstx -1];
               }
               #pragma omp parallel for
               for(int j = firstx; j < lastx; j++) {
                   writerow[j] = readrow[j];
               }
               ywrite++;
            /* Else if this is in the 'bottom' segment of the ROI-polygon */
            } else if(i >= (*(*corners + 1)).y) {
                #pragma omp parallel for
                for(int j = 0; j < (*(*corners + 5)).x; j++) {
                    writerow[j] = readrow[j];
                }
                ywrite++;
            } else {
              cout << "Something went wrong while setting ROI for a three-channeled image." << endl;
            }
        }
        res = result;
      }
    }


    /*
      Point** corners expect 6 points.  {bottom left, middle left, top left, top right, middle right, bottom right}.
      Points should for a trapezoid shape.

      example of points, pp is what is given to the method: 
      cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
      cv::Point * pp[1] = {roi_corners[0]};

      Mat img is assumed to be a triple-channel RBG image.

      Mat res is a grayscaled image cropped to the appropriate region of interest.      
    */
    void setROIColorImageGray(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res) {

      /* If anything other than 6 points have been supplied, reject */
      if(numpoint != 6) {
        cout << "Usage: supply 6 points to setROI(Point **corners, int numpoints, Mat & img, Mat & res). Anything else will be rejected." << endl;
        exit(0);
      } else {
        /* Calculate dimensions and initialize resulting cv::Mat */
        int x = (*(*corners + 5)).x;
        int y = img.rows - (((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y);
        cv::Mat result = cv::Mat::zeros(y, x, CV_8UC1);

        double firstSlope, firstIntersect, secondSlope, secondIntersect;
        firstSlope  = ((double)(*(*corners + 2)).y - (double)(*(*corners + 1)).y) / ((double)(*(*corners + 2)).x - (double)((*(*corners + 1)).x));
        secondSlope = ((double)(*(*corners + 2)).y - (double)(*(*corners + 4)).y) / ((double)(*(*corners + 2)).x - ((double)(*(*corners + 4)).x));
        firstIntersect  = (double)(*(*corners + 1)).y - (firstSlope * (double)(*(*corners + 1)).x);
        secondIntersect = (double)(*(*corners + 4)).y - (secondSlope * (double)(*(*corners + 4)).x);

        /* Calculate the different indexes to read and write from in the original cv::Mat */
        int ymin = ((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y;
        int ymax = img.rows;
        int ywrite = 0;
        #pragma omp parallel for
        for(int i = ymin; i < ymax; i++) {
            cv::Vec3b* readrow = img.ptr<cv::Vec3b>(i);
            unsigned char* writerow = result.ptr<unsigned char>(ywrite);
            
            /* If this row is in the 'top' segment of the ROI-polygon */
            if(i < (*(*corners + 1)).y) {
               int firstx = (i - firstIntersect) / firstSlope;
               int lastx = (i - secondIntersect) / secondSlope;
               if(firstx == lastx) {
                   cv::Vec3b pixels = readrow[firstx -1];
                   writerow[firstx -1] = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
               }
               #pragma omp parallel for
               for(int j = firstx; j < lastx; j++) {
                   cv::Vec3b pixels = readrow[j];
                   writerow[j] = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
               }
               ywrite++;
            /* Else if this is in the 'bottom' segment of the ROI-polygon */
            } else if(i >= (*(*corners + 1)).y) {
               #pragma omp parallel for
                for(int j = 0; j < (*(*corners + 5)).x; j++) {
                    cv::Vec3b pixels = readrow[j];
                    writerow[j] = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
                }
                ywrite++;
            } else {
              cout << "Something went wrong while setting ROI for a three-channeled image." << endl;
            }
        }
        res = result;
      }
    }

    /*
      Point** corners expect 6 points.  {bottom left, middle left, top left, top right, middle right, bottom right}.
      Points should for a trapezoid shape.

      example of points, pp is what is given to the method: 
      cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
      cv::Point * pp[1] = {roi_corners[0]};

      Mat img is assumed to be a triple-channel RBG image.

      Mat res is a binarized image cropped to the appropriate region of interest.      
    */
    void setROIColorImageBin(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res) {

      /* If anything other than 6 points have been supplied, reject */
      if(numpoint != 6) {
        cout << "Usage: supply 6 points to setROI(Point **corners, int numpoints, Mat & img, Mat & res). Anything else will be rejected." << endl;
        exit(0);
      } else {
        /* Calculate dimensions and initialize resulting cv::Mat */
        int x = (*(*corners + 5)).x;
        int y = img.rows - (((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y);
        cv::Mat result = cv::Mat::zeros(y, x, CV_8UC1);

        double firstSlope, firstIntersect, secondSlope, secondIntersect;
        firstSlope  = ((double)(*(*corners + 2)).y - (double)(*(*corners + 1)).y) / ((double)(*(*corners + 2)).x - (double)((*(*corners + 1)).x));
        secondSlope = ((double)(*(*corners + 2)).y - (double)(*(*corners + 4)).y) / ((double)(*(*corners + 2)).x - ((double)(*(*corners + 4)).x));
        firstIntersect  = (double)(*(*corners + 1)).y - (firstSlope * (double)(*(*corners + 1)).x);
        secondIntersect = (double)(*(*corners + 4)).y - (secondSlope * (double)(*(*corners + 4)).x);

        /* Calculate the different indexes to read and write from in the original cv::Mat */
        int ymin = ((*(*corners + 2)).y < (*(*corners + 3)).y) ? (*(*corners + 3)).y: (*(*corners + 2)).y;
        int ymax = img.rows;
        int ywrite = 0;
        #pragma omp parallel for
        for(int i = ymin; i < ymax; i++) {
            cv::Vec3b* readrow = img.ptr<cv::Vec3b>(i);
            unsigned char* writerow = result.ptr<unsigned char>(ywrite);
            
            /* If this row is in the 'top' segment of the ROI-polygon */
            if(i < (*(*corners + 1)).y) {
               int firstx = (i - firstIntersect) / firstSlope;
               int lastx = (i - secondIntersect) / secondSlope;
               if(firstx == lastx) {
                   cv::Vec3b pixels = readrow[firstx -1];
                   unsigned char gray = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
                   writerow[firstx -1] = (gray < 128) ? 0 : 255;
               }
               #pragma omp parallel for
               for(int j = firstx; j < lastx; j++) {
                   cv::Vec3b pixels = readrow[j];
                   unsigned char gray = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
                   writerow[j] = (gray < 128) ? 0 : 255;
               }
               ywrite++;
            /* Else if this is in the 'bottom' segment of the ROI-polygon */
            } else if(i >= (*(*corners + 1)).y) {
                #pragma omp parallel for
                for(int j = 0; j < (*(*corners + 5)).x; j++) {
                    cv::Vec3b pixels = readrow[j];
                    unsigned char gray = (unsigned char)((0.21 * pixels[2]) + (0.72 * pixels[1]) + (0.07 * pixels[0]));
                    writerow[j] = (gray < 128) ? 0 : 255;
                }
                ywrite++;
            } else {
              cout << "Something went wrong while setting ROI for a three-channeled image." << endl;
            }
        }
        res = result;
      }
    }
}

