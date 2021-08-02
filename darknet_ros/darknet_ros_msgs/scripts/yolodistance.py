#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import pyrealsense2 as rs
import numpy as np
import rospy
from rospy.topics import Subscriber
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
import sys
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray



bridge = CvBridge() #11 28
global lis_socketx, lis_sockety, depth_array, lis_switchx, lis_switchy, lis_satellitex, lis_satellitey, lis_dimmerx, lis_dimmery,socomponents_locations, swcomponents_locations, sacomponents_locations,dicomponents_locations
socomponents_locations = [0.0,0.0,0.0,0.0]
swcomponents_locations = [0.0,0.0,0.0,0.0]
sacomponents_locations = [0.0,0.0,0.0,0.0]
dicomponents_locations = [0.0,0.0,0.0,0.0]

def callback(data):
    
    global _intrinsics
    if data.bounding_boxes is not None:
        for i in range(len(data.bounding_boxes)):
            if data.bounding_boxes[i].Class == 'socket':
                #print("Socket: ",len(data.bounding_boxes[i].Class))
                global lis_socketx
                global lis_sockety
                global depth_array
                global socomponents_locations 
                lis_socketx = []
                lis_sockety = []
                lis_socketx.append(data.bounding_boxes[i].xmin)
                lis_socketx.append(data.bounding_boxes[i].xmax)
                lis_sockety.append(data.bounding_boxes[i].ymin)
                lis_sockety.append(data.bounding_boxes[i].ymax)
                #if (lis_socketx and lis_sockety) is not None:
                socket_cx = int((lis_socketx[0] + lis_socketx[1])/2)
                socket_cy = int((lis_sockety[0] + lis_sockety[1])/2)
                
                distance = depth_array[int((lis_sockety[0] + lis_sockety[1])/2), int((lis_socketx[0] + lis_socketx[1])/2)]
                #distance = depth_array[int((lis_socketx[0] + lis_socketx[1])/2),int((lis_sockety[0] + lis_sockety[1])/2) ]
                depth_point_socket = rs.rs2_deproject_pixel_to_point(_intrinsics, (socket_cx, socket_cy), distance/1000)
                
                socomponents_locations = [depth_point_socket[0], depth_point_socket[2], -depth_point_socket[1],1]
                #print('Socket distance: ',depth_point_socket)
                rospy.loginfo("Socket position: X:%s Y:%s Z:%s " %(depth_point_socket[0],depth_point_socket[1],depth_point_socket[2]))
            if data.bounding_boxes[i].Class == 'switch':
                #print("Switch:", len(data.bounding_boxes[i]))
                global lis_switchx
                global lis_switchy
                global depth_array
                global swcomponents_locations 
                lis_switchx = []
                lis_switchy = []
                lis_switchx.append(data.bounding_boxes[i].xmin)
                lis_switchx.append(data.bounding_boxes[i].xmax)
                lis_switchy.append(data.bounding_boxes[i].ymin)
                lis_switchy.append(data.bounding_boxes[i].ymax)
                #print(lis_switchx)
                
                    #print([int((lis_switchx[0] + lis_switchx[1])/2), int((lis_switchy[0] + lis_switchy[1])/2)])
                
                distance = depth_array[int((lis_switchy[0] + lis_switchy[1])/2), int((lis_switchx[0] + lis_switchx[1])/2)]
                #distance = depth_array[int((lis_switchx[0] + lis_switchx[1])/2),int((lis_switchy[0] + lis_switchy[1])/2) ]
                switch_cx = int((lis_switchx[0] + lis_switchx[1])/2)
                switch_cy = int((lis_switchy[0] + lis_switchy[1])/2)
                
            
                depth_point_switch = rs.rs2_deproject_pixel_to_point(_intrinsics, (switch_cx, switch_cy), distance/1000)
                swcomponents_locations = [depth_point_switch[0], depth_point_switch[2], depth_point_switch[1],2]
                #print('Switch distance: ',depth_point_switch)
                rospy.loginfo("Switch position: X:%s Y:%s Z:%s " %(depth_point_switch[0],depth_point_switch[1],depth_point_switch[2]))
                

            if data.bounding_boxes[i].Class == 'satellite input':
                #print("Sat Input:",len(data.bounding_boxes[i].Class))
                global lis_satellitex
                global lis_satellitey
                global depth_array
                global sacomponents_locations  
                lis_satellitex = []
                lis_satellitey = []
                lis_satellitex.append(data.bounding_boxes[i].xmin)
                lis_satellitex.append(data.bounding_boxes[i].xmax)
                lis_satellitey.append(data.bounding_boxes[i].ymin)
                lis_satellitey.append(data.bounding_boxes[i].ymax)
                #if (lis_satellitex and lis_satellitey) is not None:
                
                distance = depth_array[int((lis_satellitey[0] + lis_satellitey[1])/2), int((lis_satellitex[0] + lis_satellitex[1])/2)]
                #distance = depth_array[int((lis_satellitex[0] + lis_satellitex[1])/2),int((lis_satellitey[0] + lis_satellitey[1])/2) ]
                satellite_cx = int((lis_satellitex[0] + lis_satellitex[1])/2)
                satellite_cy = int((lis_satellitey[0] + lis_satellitey[1])/2)
                depth_point_satellite = rs.rs2_deproject_pixel_to_point(_intrinsics, (satellite_cx, satellite_cy), distance/1000)
                sacomponents_locations = [depth_point_satellite[0], depth_point_satellite[2], depth_point_satellite[1],3]
                #print('Satellite distance: ',depth_point_satellite)
                rospy.loginfo("Satellite position: X:%s Y:%s Z:%s " %(depth_point_satellite[0],depth_point_satellite[1],depth_point_satellite[2]))
                
            if data.bounding_boxes[i].Class == 'dimmer':
                #print("Dimmer:",len(data.bounding_boxes[i].Class))
                global lis_dimmerx
                global lis_dimmery
                global depth_array
                global dicomponents_locations  
                lis_dimmerx = []
                lis_dimmery = []
                lis_dimmerx.append(data.bounding_boxes[i].xmin)
                lis_dimmerx.append(data.bounding_boxes[i].xmax)
                lis_dimmery.append(data.bounding_boxes[i].ymin)
                lis_dimmery.append(data.bounding_boxes[i].ymax)
                #if (lis_dimmerx and lis_dimmery) is not None:
                
                distance = depth_array[int((lis_dimmery[0] + lis_dimmery[1])/2), int((lis_dimmerx[0] + lis_dimmerx[1])/2)]
                #distance = depth_array[ int((lis_dimmerx[0] + lis_dimmerx[1])/2),int((lis_dimmery[0] + lis_dimmery[1])/2)]
                dimmer_cx = int((lis_dimmerx[0] + lis_dimmerx[1])/2)
                dimmer_cy = int((lis_dimmery[0] + lis_dimmery[1])/2)
                depth_point_dimmer = rs.rs2_deproject_pixel_to_point(_intrinsics, (dimmer_cx, dimmer_cy), distance/1000)
                dicomponents_locations = [depth_point_dimmer[0], depth_point_dimmer[2], depth_point_dimmer[1],4]
                #print('Dimmer distance: ',depth_point_dimmer)
                rospy.loginfo("Dimmer position: X:%s Y:%s Z:%s " %(depth_point_dimmer[0],depth_point_dimmer[1],depth_point_dimmer[2]))
                   
    

def depth_image_turn(ros_depht):
    bridge = CvBridge()
    global depth_array

     
    try:
    
        depth_image = bridge.imgmsg_to_cv2(ros_depht, desired_encoding="passthrough")
        #depth_intrin = depth_image.profile.as_video_stream_profile().intrinsics
    except CvBridgeError as e:
 	    print(e)
     #Convert the depth image to a Numpy array
    
    depth_array = np.array(depth_image, dtype=np.float32)

    #rospy.loginfo(depth_array)
    #cv2.imshow('Depht',depth_array)
    depth_im = np.asanyarray(depth_array)
    
    #print(distance)

"""
def image_turn(ros_image):
    #print('Image taken!')
    global bridge,socomponents_locations,swcomponents_locations,sacomponents_locations,dicomponents_locations

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    global lis_socketx
    global lis_sockety
    global lis_switchx
    global lis_switchy
    global lis_dimmerx 
    global lis_dimmery
    global lis_satellitex
    global lis_satellitey
    #cv2.circle(cv_image, (int((lis_socketx[0] + lis_socketx[1])/2),int((lis_sockety[0] + lis_sockety[1])/2)), radius=1, color=(0, 0, 255), thickness=5)
    #cv2.circle(cv_image, (int((lis_switchx[0] + lis_switchx[1])/2),int((lis_switchy[0] + lis_switchy[1])/2)), radius=1, color=(0, 0, 255), thickness=5)
    #cv2.putText(cv_image, "{0:.2f}m".format(socomponents_locations[1]), (lis_socketx[1], lis_sockety[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    #cv2.putText(cv_image, "{0:.2f}m".format(swcomponents_locations[1]), (lis_switchx[1], lis_switchy[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    #cv2.putText(cv_image, "{}mm".format(sacomponents_locations[2]), (lis_satellitex[1], lis_satellitey[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    #cv2.putText(cv_image, "{}mm".format(dicomponents_locations[1]), (lis_dimmerx[1], lis_dimmery[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    #cv2.imshow('Camera',cv_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('exit...')
"""
def imageDepthInfoCallback(cameraInfo):
    global _intrinsics
    _intrinsics = rs.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    #result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    #result[0]: right, result[1]: down, result[2]: forward
    
def main(args):
    global components_locations
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, callback)
    #rospy.Subscriber('/camera/color/image_raw',Image,image_turn)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image,depth_image_turn)
    rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, imageDepthInfoCallback)
    components = rospy.Publisher('auto_arm_topic', Float64MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        
        so_message = Float64MultiArray(data=socomponents_locations)
        sw_message = Float64MultiArray(data=swcomponents_locations)
        sa_message = Float64MultiArray(data=sacomponents_locations)
        di_message = Float64MultiArray(data=dicomponents_locations)
        components.publish(so_message)
        components.publish(sw_message)
        components.publish(sa_message)
        components.publish(di_message)
        rospy.sleep(1)
            #count = 0
        #count +=1 
        #rate.sleep(10)
    """
    while count == 0:
        so_message = Float64MultiArray(data=socomponents_locations)
        sw_message = Float64MultiArray(data=swcomponents_locations)
        sa_message = Float64MultiArray(data=sacomponents_locations)
        di_message = Float64MultiArray(data=dicomponents_locations)
        #if count == 15:
        components.publish(sw_message)
        count = 1
    """
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Closed')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)