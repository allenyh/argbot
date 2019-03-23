#!/usr/bin/python

import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import numpy as np
import glob, os
import tf
import struct
import math
import time
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
from cv_bridge import CvBridge, CvBridgeError

class ImageCreator():


    def __init__(self):
        self.bin = 360
        self.range_max = 5.5
        self.border = 10
        self.scale = 7.
        self.point_size = 2

        self.image_num = 0
        self.bridge = CvBridge()        
        self.folder = '/home/allen/subt_docker/bag' #write your folder name betweer ' /'

        #---bag part---
        os.chdir(self.folder)
        for folder in os.listdir('.'):
            self.img_num = 0

            for file in glob.glob(folder+"/*.bag"):
                with rosbag.Bag(file, 'r') as bag: #open first .bag
                    print (file)
                    for topic,msg,t in bag.read_messages():
                        self.range_max = msg.range_max
                        self.bin = len(msg.ranges)
                        self.init_param()

                        for i in range(self.bin):
                            if msg.ranges[i] != float("inf"):
                                rad = (i/360.)*2.*math.pi/2.
                                x = self.scale*msg.ranges[i] * np.cos(rad)
                                y = self.scale*msg.ranges[i] * np.sin(rad)
                                x_ = int(x + self.width/2.)
                                y_ = int(y + self.height/2.)
                                self.img[x_][y_] = 255
                                self.img = self.img_dilate(self.img, x_, y_)
                        image_name = str(self.image_num)+ ".jpg"
                        cv2.imwrite(self.folder_name+folder+image_name, self.img)
                        self.image_num += 1

    def init_param(self):
        self.width = int(self.range_max*self.scale*2 + self.border*2)
        self.height = int(self.range_max*self.scale*2 + self.border*2)
        self.img = np.zeros((int(self.height), int(self.width)), np.uint8)


if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
