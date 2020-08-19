#!/usr/bin/env python

import os, sys
import rospy
import rospkg
import subprocess

import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf


class AmclWrapper(object):

	# description: an external manager to load specific lidar map 
	# and run amcl localization when GPS fails for a long time
	def __init__(self, frequency=50, time_thresh=2):
		# frequency: running frequency for the wrapper
		# time_thresh: when GPS data fail for a period larger than time_thresh, launch amcl

		self.frequency = frequency
		self.time_thresh = time_thresh

		self.rate = rospy.Rate(self.frequency)

		self.sub_gps = rospy.Subscriber("/odometry/gps", Odometry, self.gps_callback)
		self.sub_filter = rospy.Subscriber("/odometry/filtered", Odometry, self.filter_callback)
		self.pub = rospy.Publisher("/amcl_initialpose", PoseWithCovarianceStamped, queue_size=10)

		self.amcl_running = False

	def run(self):
		while not rospy.is_shutdown():
			time_curr = rospy.Time.now().to_sec()
			if not (hasattr(self, "gps_last_time") and hasattr(self, "last_loc")):
				continue

			if (time_curr - self.gps_last_time > self.time_thresh) and (not self.amcl_running):
				print("start to add AMCL lidar-based localization")
				if self.load_lidarmap():
					self.run_amcl()
			elif (time_curr - self.gps_last_time <= self.time_thresh) and self.amcl_running:
				node_name = "amcl"
				cmd_kill = "rosnode kill %s" % node_name
				subprocess.call(cmd_kill, shell=True)
				print("stop running amcl.")

	def gps_callback(self, msg_gps):
		self.gps_last_time = msg_gps.header.stamp.to_sec()

	def filter_callback(self, msg_filter):
		self.last_loc = np.array([msg_filter.pose.pose.position.x, msg_filter.pose.pose.position.y])
		#quaternion = msg_filter.pose.pose.
		#self.last_heading = 

	def load_lidarmap(self):
		if np.linalg.norm(self.last_loc - np.array([55, 45])) < 80:
			self.load_named_map("hackerman")
			return True
		elif np.linalg.norm(self.last_loc - np.array([25, 300])) < 150:
			self.load_named_map("gilman")
			return True
		else:
			print("no associated maps available.")
			return False

	def load_named_map(self, map_name):
		print("loading %s map" % map_name)
		cmd = "roslaunch ugv_localization rampage_map_n_static_tf.launch lidarmap:=%s" % map_name
		subprocess.call(cmd, shell=True)
		print("map loaded.")

	def run_acml(self):
		print("running amcl localizer ...")
		self.init_amcl_pose()
		cmd = "roslaunch ugv_localization rampage_amcl_diff.launch"
		self.amcl_running = True

	def init_amcl_pose(self):
		t = tf.TransformerROS()
		#(w2o_trans, w2o_rot) = t.lookupTransform("world", "og_org", t.getLatestCommonTime("world", "og_org"))
		(w2o_trans, w2o_rot) = t.lookupTransform("og_org", "base_link", t.getLatestCommonTime("og_org", "base_link"))
		print(w2o_trans)
		#tf_w2o = tf.fromTranslationRotation(w2o_trans, w2o_rot)
		#tf_o2w = tf.transformations.inverse_matrix(tf_w2o)


if __name__ == "__main__":

	rospy.init_node('amcl_wrapper', anonymous=True)

	amcl_wrapper = AmclWrapper()
	amcl_wrapper.run()

	rospy.spin()



