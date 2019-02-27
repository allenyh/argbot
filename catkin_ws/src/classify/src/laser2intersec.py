#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from math import degrees, radians, atan2
import rospkg
from cv_bridge import CvBridge, CvBridgeError

class laser2intersec():
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		rospy.Subscriber('/X1/scan', LaserScan, self.call_back, queue_size = 1, buff_size = 2**24)
		self.pub = rospy.Publisher('/X1/intersection_points', Float32MultiArray, queue_size=1)

		self.bin = 360
		self.range_max = 5.5
		self.border = 10
		self.scale = 50.
		self.neighbor_dist_thres = 3

	def init_param(self):
		self.width = int(self.range_max*self.scale*2 + self.border*2)
		self.height = int(self.range_max*self.scale*2 + self.border*2)
		self.img = np.zeros((int(self.height), int(self.width)), np.uint8)

	def call_back(self, msg):
		print("\n")
		gap_start = []
		gap_end = []
		middle = []
		self.range_max = msg.range_max
		self.bin = len(msg.ranges)
		self.init_param()

		for i in range(self.bin):
			if msg.ranges[i] != float("inf"):
				rad = (i/float(self.bin))*2.*math.pi
				x = self.scale*msg.ranges[i] * np.cos(rad)
				y = self.scale*msg.ranges[i] * np.sin(rad)
				self.img[int(x + self.width/2.)][int(y + self.height/2.)] = 255

				if i != self.bin-1:
					if msg.ranges[i+1] == float("inf"):
						#print("append_start")
						gap_start.append([degrees(rad), msg.ranges[i]])

					#detect if there's neighboring points dist. diff larger than thres.
					elif abs(msg.ranges[i] - msg.ranges[i+1]) > self.neighbor_dist_thres:
						#print("neighbor", abs(msg.ranges[i] - msg.ranges[i+1]))
						gap_start.append([degrees(rad), msg.ranges[i]])
						rad_next = ((i+1)/float(self.bin))*2.*math.pi
						gap_end.append([degrees(rad_next), msg.ranges[i+1]])

			else:#"inf" case
				if i != self.bin-1 and msg.ranges[i+1] != float("inf"):
					rad = ((i+1)/float(self.bin))*2.*math.pi
					#print("append end")
					gap_end.append([degrees(rad), msg.ranges[i+1]])

		#shift gap_start if detect 'end' first instead of 'start'
		if msg.ranges[0] == float("inf"):
			start = np.roll(gap_start,2)

		if len(gap_start)-len(gap_end) != 0:
			#print("start", len(gap_start))
			#print("end", len(gap_end))
			if len(gap_start)-len(gap_end) == 1:
				self.find_change(msg, gap_end, flag = "end")
			elif len(gap_end) - len(gap_start) == 1:
				self.find_change(msg, gap_start, flag = "start")

		#when it comes to here, the length of gap_start and gap_end should be the same and organized
		if len(gap_start) != len(gap_end):
			rospy.loginfo("length mismatch!!!")
			print("start", len(gap_start))
			print("end", len(gap_end))
		for i in range(len(gap_start)):
			x1 = gap_end[i][1] * np.cos(radians(gap_end[i][0]))
			y1 = gap_end[i][1] * np.sin(radians(gap_end[i][0]))
			x2 = gap_start[i][1] * np.cos(radians(gap_start[i][0]))
			y2 = gap_start[i][1] * np.sin(radians(gap_start[i][0]))
			dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
			if dist >= 5:
				middle.append([degrees(atan2((y2+y1)/2, (x2+x1)/2)), (gap_start[i][1]+gap_end[i][1])/2])
		result = Float32MultiArray()
		for i in range(len(middle)):
			#print(middle[i][0], middle[i][1])
			result.data.append(middle[i][0])
			result.data.append(middle[i][1])
		self.pub.publish(result)

	def find_change(self, msg, l, flag):
		gap_find = None 
		for i in range(len(msg.ranges)):
			if i == len(msg.ranges)-1:
				print("end")
			d_now = msg.ranges[i]
			d_next = msg.ranges[i+1]
			if gap_find == None:
				if d_now != float("inf") and d_next == float("inf"):
					gap_find = "start"
				elif d_now == float("inf") and d_next != float("inf"):
					gap_find = "end"

			else:#find_flag become True in last iteration
				if flag == "start" and flag == gap_find:
					rad = ((i-1)/float(len(msg.ranges)))*2.*math.pi
					print("append start")
					l.append([degrees(rad), msg.ranges[i-1]])
					return
					
				elif flag == "end" and flag == gap_find:
					rad = (i/float(len(msg.ranges)))*2.*math.pi
					print("append end")
					l.append([degrees(rad), msg.ranges[i]])
					return
				else:# flag != gap_find
					i = None
					if flag == "start":
						i = len(msg.ranges)-1
					else:
						i = 0
					rad = (i/len(msg.ranges))*2.*math.pi
					print("append l")
					l.append([degrees(rad), msg.ranges[i]])
					return

if __name__ == '__main__':
	rospy.init_node('laser2intersec')
	foo = laser2intersec()
	rospy.spin()