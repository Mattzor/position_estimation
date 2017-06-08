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
#include "../laneDetection.h"

LaneDetection ld;
void close_operation(int s);

int main(int argc, char **argv)
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = close_operation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);


  /*
    Initiates a node called lane_error_publisher, that publishes messages to truck_cmd. This is purely for testing, and in the end this node
    should only publish the error, and if possible, x & y coordinates to the controller node.
  */
  ros::init(argc, argv, "lane_error_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<position_estimation::AckermannDrive>("truck_cmd", 1);

  /*
    Opens the camera once
  */
  ld.openCap();
  while (ros::ok())
  {
    position_estimation::AckermannDrive msg;
    double error = ld.runLD();
    if(error != 0) {
      msg.steering_angle = (error < 0) ? 0.1 : -0.1 ;
      msg.speed = 49;
      ROS_INFO("error %f", error);
      chatter_pub.publish(msg);
      ros::spinOnce();
    }
  }
  return 0;
}

/* Closes the camera in case of an interrupt */
void close_operation(int s) {
    ld.closeCap();
    exit(1);
}
