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
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "custom_msgs/GulliViewPositions.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

/* This is the last message posted by gulliview_server, the actual measured position */
custom_msgs::GulliViewPositions position;
Mat frame;
bool existmessage;

void close_operation(int s);

/*
  Every time a position is reported by the position estimation group, this node will print it out
  aswell as print the latest gulliview position.
*/
void callback(const custom_msgs::GulliViewPositions p) {
  if(existmessage) {
    int x,y;
    Point po;

    /* Plot the point given by position estimation in black */
    x = p.p1.x/10;
    y = p.p1.y/10;
    po = Point(x,y);
    circle(frame, po, 5, Scalar(0), -1, 8);

    /* Plot the point given by gulliview, this should be light gray */
    x = position.p1.x/10;
    y = position.p1.y/10;
    po = Point(x,y);
    circle(frame, po, 5, Scalar(200), -1, 8);

    /* Load the image to the screen */
    imshow("Window", frame); 
  } else {
    ROS_INFO("Exist no gulliview message yet..");
  }
}

/*
  Writing the position to the same slot makes sure that we only get the most recent one, always.
*/
void gulliback(const custom_msgs::GulliViewPositions p) {
  position = p;
  existmessage = true;
}


int main(int argc, char **argv)
{
  ROS_INFO("Starting initialization of interrupt handler..");
  /*
    Set up interrupt handler, this will close all streams when executed.
  */
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = close_operation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  ROS_INFO("Interrupt handler initialized properly..");

  existmessage = false;

  ROS_INFO("Setting up printing environment..");
  Mat m;
  m = Mat(500, 900, CV_8UC3, Scalar(255));
  frame = m;
  namedWindow("Window", WINDOW_AUTOSIZE);
  ROS_INFO("Printing environment set up successfully..");

  /*
    Launch the two nodes.
  */
  ROS_INFO("Launching nodes.."); 
  ros::init(argc, argv, "data_print");
  ros::NodeHandle n;
  ros::Subscriber data_sub = n.subscribe("kalman_topic",1000,callback);
  ROS_INFO("posest_positions subscriber successfully launched.."); 
  ros::Subscriber gulli_sub = n.subscribe("gv_positions",1,gulliback);
  ROS_INFO("gv_positions subscriber successfully launched.."); 

  waitKey(1);

  ros::spin();
  return 0;
}

/*
  Interrupt handler.
*/
void close_operation(int s) {
    destroyAllWindows();
    exit(1);
}
