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
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# range of yellow color in HSV
lower_yellow = np.array([20,120,120])
upper_yellow = np.array([40,255,255])

font = cv2.FONT_HERSHEY_SIMPLEX
pixel_thresh = 9000

while (cap.isOpened()):
    ret, frame = cap.read()
    if (ret):
        # Convert from rgb to hsv for easy filtering
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # inRange to only capture values between lower/upper yellow
        mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        # Bitwise AND of mask and webcam feed
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # Count yellow pixels found
        nzCount = cv2.countNonZero(mask)
        
        cv2.putText(frame,'Yellow pixel count: ',(20,30), font, 1,(255,0,0),1,cv2.LINE_AA)
        cv2.putText(frame, str(nzCount),(20,80), font, 2,(255,0,0),1,cv2.LINE_AA)

        if nzCount > pixel_thresh:
            cv2.putText(frame,'Truck at turn! ',(100,200), font, 2,(0,0,255),8,cv2.LINE_AA)
        cv2.imshow('hsv_img', hsv_img)
        cv2.imshow('mask', mask)
        cv2.imshow('Filtered color only', res)
        cv2.imshow('Webcam',frame)
    
    if cv2.waitKey(1) == 13: #Enter
        break
        
cap.release()
cv2.destroyAllWindows()