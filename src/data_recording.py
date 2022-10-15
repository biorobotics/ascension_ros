#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np

class data_recording:
	def __init__(self):
		rospy.Subscriber('/ascension_node/target_poses',TransformStamped, self.callback_pose, queue_size=1)

		self.x = None
		self.y = None
		self.z = None
		self.wx = None
		self.wy = None
		self.wz = None
		self.w = None

		# self.file = open('/Home/trakSTAR_ws/src/ascension_ros/pose_data/data_1.txt','w')

		self.file = open('./data_circle2.txt','w')

		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			if self.x is not None:
				line = str(self.x)+"\t" + str(self.y)+"\t" +str(self.z)+"\t" + \
				str(self.wx)+"\t" + str(self.wy)+"\t" +str(self.wz)+"\t" + str(self.w) + '\n'
				
				self.file.write(line)
				print('self.x',self.x)
				rate.sleep()
		# self.file.close()


	def callback_pose(self,data):
		self.x = data.transform.translation.x
		self.y = data.transform.translation.y
		self.z = data.transform.translation.z
		self.wx = data.transform.rotation.x
		self.wy = data.transform.rotation.y
		self.wz = data.transform.rotation.z
		self.w = data.transform.rotation.w

if __name__== '__main__':
	rospy.init_node('data_recording')
	node = data_recording()