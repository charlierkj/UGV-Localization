#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped


class WaypointsSaver(object):

	def __init__(self, topic, max_num=30):
		self.topic = topic
		self.max_num = max_num

		self.sub = rospy.Subscriber(topic, PointStamped, self.callback)
		self.waypoints = np.empty(shape=(0, 2)) # x, y

	def callback(self, msg_wp):
		if self.waypoints.shape[0] < self.max_num:
			record = np.array([[msg_wp.point.x, msg_wp.point.y]])
			self.waypoints = np.vstack((self.waypoints, record))


	def save_npy(self, path):
		np.save(path, self.waypoints)


if __name__ == "__main__":

	rospy.init_node('click_waypoints', anonymous=True)

	topic = "mapviz/clicked_point"
	npy_path = "/home/charlierkj/asco/src/ugv_localization/records/wps.npy"

	waypoints_saver = WaypointsSaver(topic)
	rospy.spin()

	# if rospy.is_shutdown():
	#	waypoints_saver.save_npy(npy_path)
	

