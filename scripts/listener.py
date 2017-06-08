#!/usr/bin/env python
"""
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
"""

import rospy
import cv2
import numpy as np
#from custom_msgs import *
#from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive

cap = cv2.VideoCapture(0)

pp = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi', pp, 20.0, (640,480))
text_file = open("ackermann_values.txt", "w")

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    if cap.isOpened():
        ret, frame = cap.read()
        if (ret):
            out.write(frame)
            text_file.write("%f  %f\n"%(data.steering_angle, data.speed))
        else:
            rospy.loginfo("could not create frame\n")   
    else:
        rospy.loginfo("cam not opened")            


def gulliCallback(data):
    text_file.write("%f %f %f\n"%(data.p.x, data.p.y, data.theta1))


def shutdown_ros():
   text_file.close()
   cap.release()
   out.release()


def listener():
#    import os
#    os.system("sudo modprobe bcm2835-v4l2")
    rospy.init_node("listener", anonymous = True)
#    rospy.Subscriber("ACtopic", AckermannDrive, callback)
    rospy.Subscriber("master_drive", AckermannDrive, callback)
#    rospy.Subscriber("truck_state", TruckState, gulliCallback)
    rospy.on_shutdown(shutdown_ros)
    rospy.loginfo("started listener node")
    rospy.spin()
    

if __name__ == '__main__':
    listener()
    #cap.release()
    #out.release()
    #text_file.close()   
