'''
Lucas-Kanade optical flow tracker
=============
Takes an input video stream and outputs a video with inverted perspective and with overlayed anchor points tracking.
Also saves the computed steering angle and velocity to a .txt file for each frame. During runtime the program plots
the estimated path and the computed flow vector. The program also saves the path to a .jpg file when terminated.
----
Input video: 'video/demo_route.mp4', change this to whatever
Output video: 'output_video.avi'
Output .txt file: 'output_data.txt', contains estimated steering angle and speed
Output track image: 'map.jpg'
----
Commands:
ESC - exit
Enter - pause
Space - step frame by frame
'''

# Python 2/3 compatibility
from __future__ import print_function

from scipy import polyfit

import math
import numpy as np
import cv2
import time
from time import clock

# Transforms perspective to bird's-eye view
def drawIPM(img, y_top):
    imshape = img.shape
    points_A = np.float32([[0,imshape[0]], [0,y_top], [imshape[1],y_top], [imshape[1],imshape[0]]])
    points_B = np.float32([[50.0/128*imshape[1],imshape[0]], [0,0], [imshape[1],0], [78.0/128*imshape[1],imshape[0]]])
    
    M = cv2.getPerspectiveTransform(points_A, points_B)

    warped = cv2.warpPerspective(img, M, (imshape[1], imshape[0]), flags=cv2.INTER_LINEAR)
    
    return warped

# Computes the average anchor point track (movement) from a set of all found anchor point's known tracks
def computeTrack(tracks, img):
    x_dif_tot = 0
    y_dif_tot = 0
    angle = None
    track_length = None
    k = 0
    imgshape = img.shape
    x_size = imgshape[1]
    y_size = imgshape[0]

    if tracks:
        for i, track in enumerate(tracks):
            old = track[0]
            recent = track[len(track)-1]
            
            # Scale movement depending on where the anchor point is located
            x_dif = recent[0] - old[0]
            x_dif = (1 - abs(recent[0]-x_size/2.0)/(x_size/2.0))*x_dif
            y_dif = recent[1] - old[1]
            y_dif = (1 - abs(y_size-recent[1])/y_size)*y_dif
            
            tmp_hyp = math.hypot(x_dif, y_dif)
            if y_dif > 0:
                x_dif_tot += x_dif
                y_dif_tot += y_dif
                k += 1
            
        if k != 0: 
            x_dif_tot = x_dif_tot / k
            y_dif_tot = y_dif_tot / k
            track_length = math.hypot(x_dif_tot, y_dif_tot)
            angle = math.atan(y_dif_tot / x_dif_tot)
            if x_dif_tot < 0:
                angle += math.pi
            angle += math.pi
            
    return angle, track_length

# Set last known X,Y-position and vehicle direction (angle)
def set_v(x,y,angle):
    global v_speed
    v_speed = (x,y,angle)

# Plots the estimated track of the vehicle
def drawTrack(first, angle, track_length, img_track, startPos, arrow_length, frame):
    font = cv2.FONT_HERSHEY_SIMPLEX
    if first:
        endX = int(startPos[0])
        endY = int(startPos[1] - 10)
        cv2.arrowedLine(img_track, (startPos[0], startPos[1]), (endX, endY), color=[255, 0, 0], thickness=2)
        cv2.circle(img_track,(startPos[0],startPos[1]), 10, (0,0,255), -1)
        cv2.putText(img_track,'Start ',(startPos[0] + 20,startPos[1]), font, 0.5,(0,0,255),1,cv2.LINE_AA)
        cv2.putText(img_track,str((startPos[0],startPos[1])),(startPos[0]+10,startPos[1]+20), font, 0.4,(0,0,255),1,cv2.LINE_AA)
        cv2.imshow('Track', img_track)
        set_v(endX, endY, math.pi*3/2)
    else:
        endX = int(v_speed[0] - (arrow_length)*math.cos(v_speed[2] - angle + math.pi*3/2))
        endY = int(v_speed[1] + (arrow_length)*math.sin(v_speed[2] - angle + math.pi*3/2))
        cv2.arrowedLine(img_track, (v_speed[0], v_speed[1]), (endX, endY), color=[0, 0, 0], thickness=2)
        if frame % 7 == 0:
            cv2.putText(img_track,str((endX,endY)),(endX+10,endY), font, 0.4,(0,0,255),1,cv2.LINE_AA)
        cv2.imshow('Track', img_track)
        set_v(endX, endY,  v_speed[2] - angle + math.pi*3.0/2.0 )
        
# Plots the vehicle's estimated movement vector based on all the anchor points
def drawArrow(angle, track_length, img_direction):
    endX = int(350 + (20+5*track_length)*math.cos(angle))
    endY = int(350 + (20+5*track_length)*math.sin(angle))
   
    cv2.arrowedLine(img_direction, (350, 350), (endX, endY), color=[0, 255, 0], thickness=3)
    cv2.imshow('Vehicle movement', img_direction)

# Draws overlayed text on the output video stream, # of frames and estimated steering angle
def drawText(angle, vis, font,frames):
    cv2.putText(vis,'Frames: ',(20,420), font, 0.6,(0,0,255),1,cv2.LINE_AA)
    cv2.putText(vis, str(frames),(150,420), font, 0.6,(0,0,255),1,cv2.LINE_AA)
    
    angleDeg = int(angle*180/math.pi)
    steeringAngle = -(90 - (360-angleDeg))
    cv2.putText(vis,'Steering angle: ',(20,450), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    cv2.putText(vis,str(steeringAngle),(150,450), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    
    
# Lukas-Kanade parameters
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.1,
                       minDistance = 7,
                       blockSize = 7 )

class App:
    def __init__(self, video_src):
        self.track_len = 5
        self.detect_interval = 5
        self.tracks = []
        self.cam = cv2.VideoCapture(video_src)
        self.frame_idx = 0
    

    def run(self):
        #-------Record video------------#
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output_video.avi',fourcc, 30.0, (1280,720))
        
        # Initialize image for flow vector and track
        img_direction = np.zeros((700,700), dtype=np.uint8)
        img_direction.fill(255)
        img_track = np.zeros((1000,820), dtype=np.uint8)
        img_track.fill(255)
        
        frames = 0
        first = True
        arrLength = 3
        font = cv2.FONT_HERSHEY_SIMPLEX
        stepMode = False
        
        f = open('output_data.txt','w')
        
        # FPS check
        start = time.time()
        
        # While video is opened
        while self.cam.isOpened():
            ret, frame = self.cam.read()
            # Save image of track when terminated
            if not ret:
                cv2.imwrite('map.jpg', img_track)
                break
            
            # Transform perspective
            frame = drawIPM(frame, 370.0/720*frame.shape[0])
            imgshape = frame.shape
            # Grayscale
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            vis = frame.copy()
            
            # If active anchor points exist
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
                # Plot each anchor point's movement
                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                
                # For initialization
                if frames == 0:
                    p1_old = p1
                # Plot track and flow vector
                else:
                    angle, track_length = computeTrack(self.tracks, vis)
                    if angle is not None and abs(angle) < 2*math.pi:
                        drawArrow(angle, track_length, img_direction)
                        drawText(angle,vis,font,frames)
                        img_direction.fill(255)
                        if first:
                                drawTrack(first, angle, track_length, img_track, (200,900), arrLength, self.frame_idx)
                                first = False
                        angleDeg = int(angle*180/math.pi)
                        if (angleDeg > 260 and angleDeg < 280):
                                angle = math.pi*3/2
                                drawTrack(first, angle, track_length, img_track, (0,0), arrLength, self.frame_idx)
                        elif (angleDeg > 290):
                                angle = 4.725
                                drawTrack(first, angle, track_length, img_track, (0,0), arrLength, self.frame_idx)
                        elif (angleDeg > 200 and angleDeg < 240):
                            angle = 4.69
                            drawTrack(first, angle, track_length, img_track, (0,0), arrLength, self.frame_idx) 
                            
                        # Write steering angle and speed vector length in pixels to file "of_data.txt"
                        angleConv = 360 - angleDeg
                        speedConv = int(track_length)
                        f.write('%s %s' % (angleConv, speedConv))
                        f.write('\n')
                        
                p1_old = p1
                frames += 1
            else:
                f.write('%s %s' % (0, 0))
                f.write('\n')
            
            # Adds new anchor points every specified interval
            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None:
                    for x, y in np.float32(p).reshape(-1, 2):
                        self.tracks.append([(x, y)])
            
            # Inc frame count and set new previous frame
            self.frame_idx += 1
            self.prev_gray = frame_gray
            # Write frame to file and display result
            out.write(vis)
            cv2.imshow('lk_track', vis)
            
            # Esc to terminate, Enter to pause, Space to step frame by frame
            if stepMode:
                while True:
                    ch = cv2.waitKey(0)
                    if ch == 32:
                        break
                    elif ch == 13:
                        stepMode = False
                        break
            ch = cv2.waitKey(1)
            if ch == 27:
                cv2.destroyAllWindows()
                break
            elif ch == 13:
                while True:
                    ch = cv2.waitKey(0)
                    if ch == 13:
                        break
            elif ch == 32:
                stepMode = True
                while True:
                    ch = cv2.waitKey(0)
                    if ch == 32:
                        break
                   
                
        # FPS checker end
        end = time.time()
        seconds = end - start
        fps  = self.frame_idx / seconds;
        print ('FPS: ', fps);
    # END run         


def main():
    import sys

    video_src = 'video/yellow2.mp4'

    print(__doc__)
    App(video_src).run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()