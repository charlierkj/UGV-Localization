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
	def __init__(self, frequency=50, time_thresh=3):
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
			#print(time_curr)
			if not (hasattr(self, "gps_last_time") and hasattr(self, "last_loc")):
				continue

			if (time_curr - self.gps_last_time > self.time_thresh) and (not self.amcl_running):
				print("start to add AMCL lidar-based localization")
				if self.load_lidarmap():
					self.run_amcl()
			elif (time_curr - self.gps_last_time <= self.time_thresh) and self.amcl_running:
				self.stop_amcl()

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
		#cmd = "roslaunch ugv_localization rampage_map_n_static_tf.launch lidarmap:=%s" % map_name
		#subprocess.Popen(cmd, shell=True)
		pack_path = rospkg.RosPack().get_path("ugv_localization")
		cmds = []
		cmds.append("rosparam load %s" % os.path.join(pack_path, "map", "w2o_%s.yaml" % map_name))
		cmds.append("rosparam set /map_og_original_server/frame_id og_org")
		map_path = os.path.join(pack_path, "map", "map_%s.yaml" % map_name)
		cmds.append("rosrun map_server map_server %s __name:=map_og_original_server" % map_path)
		cmds.append("rosrun tf2_ros static_transform_publisher world2og_org __name:=world_to_og_org")
		cmds.append("rosrun tf2_ros static_transform_publisher 0.45 0 0 0 0 0 1 base_link laser __name:=base_link2laser")
		for cmd in cmds:
			subprocess.Popen(cmd, shell=True)

	def run_amcl(self):
		print("running amcl localizer ...")
		#self.init_amcl_pose()
		#cmd = "roslaunch ugv_localization rampage_amcl_diff.launch"
		#subprocess.Popen(cmd, shell=True)
		pack_path = rospkg.RosPack().get_path("ugv_localization")
		amcl_params_path = os.path.join(pack_path, "config/rampage_amcl_diff.yaml")
		cmds = []
		cmds.append("rosparam load %s amcl" % amcl_params_path)
		cmds.append("rosrun amcl amcl scan:=scan initial_pose:=amcl_initialpose __name:=amcl")
		for cmd in cmds:
			subprocess.Popen(cmd, shell=True)
		self.init_amcl_pose()
		self.amcl_running = True

	def init_amcl_pose(self):
		listener = tf.TransformListener()
		#listener.waitForTransform("og_org", "base_link", rospy.Time().now(), rospy.Duration(1))
		listener.waitForTransform("world", "og_org", rospy.Time().now(), rospy.Duration(1))
		(o2b_trans, o2b_rot) = listener.lookupTransform("og_org", "base_link", rospy.Time(0))
		o2b_a = tf.transformations.euler_from_quaternion(o2b_rot)[2]
		
		cmds = []
		cmds.append("rosparam set /amcl/initial_pose_x %s" % str(float(o2b_trans[0]) + 5))
		cmds.append("rosparam set /amcl/initial_pose_y %s" % o2b_trans[1])
		cmds.append("rosparam set /amcl/initial_pose_a %s" % o2b_a)
		for cmd in cmds:
			subprocess.call(cmd, shell=True)

		#msg = PoseWithCovarianceStamped()
		#msg.header.stamp = rospy.Time().now()
		#msg.header.frame_id = "base_link"
		#msg.pose.pose.position.x = o2b_trans[0]
		#msg.pose.pose.position.y = o2b_trans[1]
		#msg.pose.pose.position.z = o2b_trans[2]
		#msg.pose.pose.orientation.x = o2b_rot[0]
		#msg.pose.pose.orientation.y = o2b_rot[1]
		#msg.pose.pose.orientation.z = o2b_rot[2]
		#msg.pose.pose.orientation.w = o2b_rot[3]
		#msg.pose.covariance[0] = 1
		#msg.pose.covariance[7] = 1
		#msg.pose.covariance[35] = 0.0685
		#self.pub.publish(msg)
		
	def stop_amcl(self):
		cmds = []
		cmds.append("rosnode kill amcl")
		cmds.append("rosnode kill map_og_original_server")
		cmds.append("rosnode kill world_to_og_org")
		for cmd in cmds:
			subprocess.call(cmd, shell=True)
		self.amcl_running = False
		print("stop running amcl.")


if __name__ == "__main__":

	rospy.init_node('amcl_wrapper', anonymous=True)

	amcl_wrapper = AmclWrapper()
	amcl_wrapper.run()

	rospy.spin()



