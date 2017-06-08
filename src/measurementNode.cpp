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
#include "DistanceMeasurement.h"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{
        //VideoWriter video("sign.avi", CV_FOURCC('M','J','P','G'),10,Size(640,480), true);
        DistanceMeasurement d;
        std_msgs::String msg;
        int counter = 0;
        ros::init(argc, argv, "distanceMeasurement");
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("feature_detection", 1000);
        ros::Rate loop_rate(20);
	d.openCap(); // uses the movie yellow2.mp4
	//namedWindow("Window", 1);
        d.linePointSettings();
        d.focalLength = -1;
        int diff = 0;
        while (ros::ok() /*&& d.cap.isOpened()*/){
	   Mat image, work_image, cannyOut;
           bool temp = d.cap.read(image);
           if(!temp){
              cout << "cam not working\n";
	      break;
	   }
	   
           work_image = image.clone();
	   d.pTransform(work_image, work_image);
           vector<vector<Point> > contours;
           d.setContours(work_image, cannyOut, contours);
	   
	   vector<float> res;
           d.drawLines(work_image, contours);
           ++counter;
           if(d.focalLength != -1 && counter >=  10){
              counter = 0;
              std::stringstream ss;
              ss  << d.focalLength << ";" << d.truckAngle << ";" << d.globalAngle;
              msg.data = ss.str();
              ROS_INFO("%s", msg.data.c_str());
	      chatter_pub.publish(msg);
           }
           //video.write(image);
           ros::spinOnce();
           
           if (waitKey(30) == 27){
              cout << "waitkey wrong\n";
    	      exit(0); 
	  }
           d.focalLength = -1;
	   //imshow("Window", work_image);
           //waitKey(0);
        }
        d.closeCap();
        return(0);
}












