/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "position_estimation/AckermannDrive.h"
#include "custom_msgs/GulliViewPositions.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <condition_variable>
#include <mutex>

using namespace cv;
using namespace std;
Mat img;
vector<custom_msgs::GulliViewPositions> position;
vector<float> gullitimes;

void close_operation(int s);

/*
  Stores positions and the time at which they were received.
*/
void storepos(const custom_msgs::GulliViewPositions p) {
  position.push_back(p);
  gullitimes.push_back(ros::Time::now().toSec());
}

int main(int argc, char **argv)
{
  /*
    Set up interrupthandler so CTRL-C works properly. -stares angrily at gulliview-
  */
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = close_operation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  /*
    Set up the frame we will be drawing on
  */

  /*
    Instantiate node and subscribe to the topic gv_positions with a buffer of 1000, and for each message call printpos.
  */
  ros::init(argc, argv, "gulliview_print");
  ros::NodeHandle n;
                                            /* Want a size 1 buffer to ensure all messages are fresh*/
  ros::Subscriber gulliview_sub = n.subscribe("gv_positions",1,storepos);

  ros::spin();
  return 0;
}

void close_operation(int s) {
    destroyAllWindows();
    exit(1);
}
