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
#include "std_msgs/Float32MultiArray.h"
#include "custom_msgs/GulliViewPositions.h"




#include <condition_variable>
#include <mutex>
#include <iostream>
#include <fstream>


using namespace std;
custom_msgs::GulliViewPositions position;
float gullitimes;

ofstream gulliview;
ofstream kalman;

bool existmessage;

void close_operation(int s);

/*
  Stores positions and the time at which they were received.
*/
void kalmancallback(const std_msgs::Float32MultiArray a) {
  if(existmessage) {
      gulliview << position.p1.x << " " << position.p1.y << endl;
      kalman << a.data[0] << " " << a.data[1] << endl;
      ROS_INFO("Stored one entry in the output logs");
  }
}

/*
  Writing the position to the same slot makes sure that we only get the most recent one, always.
*/
void gulliback(const custom_msgs::GulliViewPositions p) {
  position = p;
  gullitimes = ros::Time::now().toSec();
  existmessage = true;
}

int main(int argc, char **argv)
{
  /*
    Set up interrupt handler, this will close all streams when executed.
  */
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = close_operation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

      
  existmessage = false;

  /*
    Open the two textfiles we will write ackermann and gulliview to.
  */
  gulliview.open("gulliview.txt");
  kalman.open("kalman.txt");
  /*
    Launch the two nodes.
  */
  ros::init(argc, argv, "data_collect");
  ros::NodeHandle n;
  ros::Subscriber gulli_sub = n.subscribe("gv_positions",1,gulliback);
  ros::Subscriber kalman_sub = n.subscribe("kalman_topic",1,kalmancallback);


  ros::spin();
  return 0;
}

/*
  Interrupt handler.
*/
void close_operation(int s) {
    kalman.close();
    gulliview.close();
    exit(1);
}
