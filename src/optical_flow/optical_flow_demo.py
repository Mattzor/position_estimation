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

'''
Lucas-Kanade tracker
====================
Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack
for track initialization and back-tracking for match verification
between frames.
----
Direction: angle in degrees of estimated road direction
----
ESC - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import math
import numpy as np
import cv2
import time
from time import clock

def computeTrack(tracks, img):
    x_dif_tot = 0
    y_dif_tot = 0
    angle = None
    track_length = None
    k = 0
    scale_factor = 50
    if tracks:
        #print(tracks)
        for i, track in enumerate(tracks):
            k += 1
            old = track[0]
            recent = track[len(track)-1]
            xr = recent[0]
            yr = recent[1]
            xo = old[0]
            yo = old[1]
            x_dif = xr - xo
            y_dif = yr - yo
            tmp_hyp = math.hypot(x_dif, y_dif)
            cv2.arrowedLine(img, (xo,yo), (xr,yr), color=[0, 0, 255], thickness=2)
            
            if xr > 640:
                xr -= int((xr - 640)/640*scale_factor)
            else:
                xr += int((640 - xr)/640*scale_factor)
            
            cv2.arrowedLine(img, (xo,yo), (int(xr),yr), color=[255, 0, 0], thickness=2)
            x_dif_tot += x_dif
            y_dif_tot += y_dif
            
        x_dif_tot = x_dif_tot / k
        y_dif_tot = y_dif_tot / k
        track_length = math.hypot(x_dif_tot, y_dif_tot)

        angle = math.atan(y_dif_tot / x_dif_tot)

        if x_dif_tot < 0:
            angle += math.pi
        angle += math.pi
            
    return angle, track_length


def set_v(x,y,angle):
    global v_speed
    v_speed = (x,y,angle)

def drawTrack(first, angle, track_length, img_track, startPos, arrow_length):
    if first:
        #endX = int(startPos[0] - (arrow_length)*math.cos(angle))
        #endY = int(startPos[1] + (arrow_length)*math.sin(angle))
        endX = int(startPos[0])
        endY = int(startPos[1] - 30)
        cv2.arrowedLine(img_track, (startPos[0], startPos[1]), (endX, endY), color=[0, 255, 0], thickness=1)
        cv2.imshow('Track', img_track)
        set_v(endX, endY, math.pi*3/2)
    else:
        endX = int(v_speed[0] - (arrow_length)*math.cos(v_speed[2] - angle + math.pi*3/2))
        endY = int(v_speed[1] + (arrow_length)*math.sin(v_speed[2] - angle + math.pi*3/2))
        cv2.arrowedLine(img_track, (v_speed[0], v_speed[1]), (endX, endY), color=[0, 255, 0], thickness=1)
        cv2.imshow('Track', img_track)
        #print('Angle: ', math.degrees(angle))
        #print('OldAngle: ', math.degrees(v_speed[2]))
        #print('NewAngle: ', math.degrees(v_speed[2] - angle + math.pi*3/2))
        set_v(endX, endY,  v_speed[2] - angle + math.pi*3/2 )
        
    
    

def drawArrow(angle, track_length, img_direction):
    endX = int(150 + (20+5*track_length)*math.cos(angle))
    endY = int(150 + (20+5*track_length)*math.sin(angle))
   
    cv2.arrowedLine(img_direction, (150, 150), (endX, endY), color=[0, 255, 0], thickness=3)
    cv2.imshow('Road direction', img_direction)


def setROI(img_full, points):
        mask = np.zeros(img_full.shape, dtype=np.uint8)
        roi_corners = np.array([points], dtype=np.int32)
        channel_count = img_full.shape[2]
        ignore_mask_color =(255,)*channel_count
        cv2.fillPoly(mask, roi_corners, ignore_mask_color)
        img_roi = cv2.bitwise_and(img_full, mask)
        return img_roi

def drawText(angle, vis, font):
    angleDeg = int(angle*180/math.pi)
    cv2.putText(vis,'Direction: ',(370,130), font, 2,(0,0,255),3,cv2.LINE_AA)
    cv2.putText(vis,str(angleDeg),(700,130), font, 2,(0,0,255),3,cv2.LINE_AA)
    if angleDeg > 320:
        cv2.putText(vis,'Right Turn: ',(370,200), font, 2,(255,0,0),3,cv2.LINE_AA)
    elif angleDeg > 160 and angleDeg < 200:
        cv2.putText(vis,'Left Turn: ',(370,200), font, 2,(255,0,0),3,cv2.LINE_AA)

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

class App:
    def __init__(self, video_src):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.cam = cv2.VideoCapture(video_src)
        self.frame_idx = 0
    

    def run(self):
        img_direction = np.zeros((300,300), dtype=np.uint8)
        img_direction.fill(255)
        img_track = np.zeros((1500,1300), dtype=np.uint8)
        img_track.fill(255)
        frames = 0
        first = True
        arrLength = 3
        trackAvg = 1
        font = cv2.FONT_HERSHEY_SIMPLEX
        while self.cam.isOpened():
            ret, frame = self.cam.read()
            frame_roi = setROI(frame, [(0,720), (0,280), (1280, 280), (1280,720)])
            frame_gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()

            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
                p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                d = abs(p0-p0r).reshape(-1, 2).max(-1)
                good = d < 1
                new_tracks = []
                for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                    if not good_flag:
                        continue
                    tr.append((x, y))
                    if len(tr) > self.track_len:
                        del tr[0]
                    new_tracks.append(tr)
                    cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
                self.tracks = new_tracks
                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                # Plot speed vector
                if frames == 0:
                    p1_old = p1
                else:
                    angle, track_length = computeTrack(self.tracks, vis)
                    if angle is not None and angle < 2*3.15 and angle > -2*3.15:
                        drawArrow(angle, track_length, img_direction)
                        drawText(angle,vis,font)
                        img_direction.fill(255)
                        if (angle > 4.89 and angle < 5.41) or first:
                            if first:
                                drawTrack(first, angle, track_length, img_track, (500,1000), arrLength)
                                first = False
                            elif frames%trackAvg == 0:
                                angle = math.pi*3/2
                                drawTrack(first, angle, track_length, img_track, (400,500), arrLength)
                            
                        if (angle > math.pi*1.85):
                                angle = 4.73
                                #print('Angle: ', 180*angleAvg/math.pi)
                                drawTrack(first, angle, track_length, img_track, (400,500), arrLength)
                        elif (angle > math.pi and angle < 3.5):
                            angle = 4.68
                            drawTrack(first, angle, track_length, img_track, (400,500), arrLength)
                            
                p1_old = p1
                frames += 1

            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None:
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.tracks.append([(x, y)])
            
            self.frame_idx += 1
            self.prev_gray = frame_gray
            cv2.imshow('lk_track', vis)
            if frames == 100:
                cv2.imwrite('op_100frames.jpg', vis)
            
            #time.sleep(0.2)
            ch = cv2.waitKey(1)
            if ch == 27:
                break
            

def main():
    import sys

    video_src = 'video/yellow1.mp4'


    print(__doc__)
    #print(video_src)
    App(video_src).run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()