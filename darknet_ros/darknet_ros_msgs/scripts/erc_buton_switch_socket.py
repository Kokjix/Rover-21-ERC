#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 
import numpy as np
from numpy.lib.polynomial import polyint
#from realsense_depth import *
import pyrealsense2 as rs

#kamera= cv2.VideoCapture(2)
#dc = DepthCamera()
dimmer_cascade=cv2.CascadeClassifier('dimmer2.xml')
switch_cascade= cv2.CascadeClassifier('switch2.xml')
#switch2_cascade= cv2.CascadeClassifier('rot_switch2.xml')
#red_cascade= cv2.CascadeClassifier('indicator_red.xml')
socket_cascade= cv2.CascadeClassifier('socket_cascade3.xml')
uydu_cascade = cv2.CascadeClassifier('uydu2.xml')
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipe_profile = pipeline.start(config)

while(1):
    #ret, depth_frame, color_frame = dc.get_frame()
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    griton=cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    #griton = cv2.equalizeHist(griton)
    
    dimmer=dimmer_cascade.detectMultiScale(griton,1.1,4)
    switch=switch_cascade.detectMultiScale(griton,1.1,8)
    #switch2=switch2_cascade.detectMultiScale(griton,1.3,4)
    #red=red_cascade.detectMultiScale(griton,1.3,4)
    uydu = uydu_cascade.detectMultiScale(griton,1.1,4)
    socket=socket_cascade.detectMultiScale(griton,1.1,4)


    
    for(x,y,w,h) in dimmer:
        #print(len(switch))
        """
        if len(switch) > 0:
            for i in range(len(switch)):
                np.delete(switch,i,0)
        """
        cv2.rectangle(color_image,(x,y),(x+w,y+h),(0,255,255),3)
        cv2.putText(color_image, 'Dimmer', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
        cv2.circle(color_image, (int((x+x+w)/2),int((y+y+h)/2)), radius=1, color=(0, 0, 255), thickness=5)
        cx = int((x+x+w)/2)
        cy = int((y+y+h)/2)
        point = [cx, cy]
        distance = depth_image[point[1], point[0]]
        cv2.putText(color_image, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        
    
    for(x,y,w,h) in switch:
        """
        if len(dimmer) > 0:
            for i in range(len(dimmer)):
                np.delete(dimmer,i,0)
        """
        cv2.rectangle(color_image,(x,y),(x+w,y+h), (0,255,255),3)
        cv2.putText(color_image, 'Switch', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
        cv2.circle(color_image, (int((x+x+w)/2),int((y+y+h)/2)), radius=1, color=(0, 0, 255), thickness=5)
        cx = int((x+x+w)/2)
        cy = int((y+y+h)/2)
        point = [cx, cy]
        distance = depth_image[point[1], point[0]]
        cv2.putText(color_image, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        

    for(x,y,w,h) in uydu:
        if len(switch) > 0:
            np.delete(switch,0,0)
        if len(socket) > 0:
            #print(len(socket))
            for i in range(len(socket)):
                np.delete(socket,i,0) 
             
        cv2.rectangle(color_image,(x,y),(x+w,y+h), (0,255,255),3)
        cv2.putText(color_image, 'Uydu', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
        cv2.circle(color_image, (int((x+x+w)/2),int((y+y+h)/2)), radius=1, color=(0, 0, 255), thickness=5)
        cx = int((x+x+w)/2)
        cy = int((y+y+h)/2)
        point = [cx, cy]
        distance = depth_image[point[1], point[0]]
        cv2.putText(color_image, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        

    for (x, y, w, h) in socket:
        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 255), 3)
        cv2.putText(color_image, 'Socket', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
        cv2.circle(color_image, (int((x+x+w)/2),int((y+y+h)/2)), radius=1, color=(0, 0, 255), thickness=5)
        cx = int((x+x+w)/2)
        cy = int((y+y+h)/2)
        point = [cx, cy]
        distance = depth_image[point[1], point[0]]
        cv2.putText(color_image, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
       
        #print(cx, cy, distance)
               
        distance = distance/1000
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, (cx, cy), distance)
        print(depth_point[0], depth_point[1], depth_point[2], cx, cy)
        

    cv2.imshow('orjinal',color_image)
    cv2.imshow('depht',depth_image)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
#dc.release()
cv2.destroyAllWindows()