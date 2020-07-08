#!/usr/bin/env python

import sys
import rospy
import subprocess

import numpy as np

import message_filters

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf import transformations


class GlobalFilterWrapper(object):

	def __init__(self, topic_odom_wheel, topic_odom_gps, topic_imu, config_file):
		# configure subscriber and publisher
		self.sub_odom_wheel = message_filters.Subscriber(topic_odom_wheel, Odometry)
		self.sub_odom_gps = message_filters.Subscriber(topic_odom_gps, Odometry)
		self.sub_imu = message_filters.Subscriber(topic_imu, Imu)
		self.subs = [self.sub_odom_wheel, self.sub_odom_gps, self.sub_imu]
		
		# time synchronizer
		self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, 10, 0.1)
		self.ts.registerCallback(self.callback)

		# initial state
		self.init_state = [0.0 for _ in range(15)]
		self.ready = False # flag indicating whether initial state is set

		# config file
		self.config_file = config_file


	def callback(self, msg_wheel, msg_gps, msg_imu):
		if self.ready:
			for sub in self.subs:
				sub.unregister()
			self.run_filter()
		else:
			self.set_init_state(msg_wheel, msg_gps, msg_imu)


	def set_init_state(self, msg_wheel, msg_gps, msg_imu):
		# position
		self.init_state[0] = msg_gps.pose.pose.position.x
		self.init_state[1] = msg_gps.pose.pose.position.y
		# orientation
		q = msg_imu.orientation
		euler = transformations.euler_from_quaternion(quaternion=(q.x, q.y, q.z, q.w))
		self.init_state[5] = euler[2]
		# linear velocity
		self.init_state[6] = msg_wheel.twist.twist.linear.x
		# angular velocity
		self.init_state[11] = msg_imu.angular_velocity.z
		# linear acceleration
		self.init_state[12] = msg_imu.linear_acceleration.x

		self.ready = True


	def run_filter(self):
		cmd_param = "rosparam load %s ekf_odom_gps_imu" % self.config_file
		cmd_set = "rosparam set /ekf_odom_gps_imu/initial_state '%s'" % self.init_state
		cmd_run = "rosrun robot_localization ekf_localization_node __name:=ekf_odom_gps_imu odometry/filtered:=/odometry/final"
		cmd_list = [cmd_param, cmd_set, cmd_run]
		print('initial state: ', self.init_state)
		for cmd in cmd_list:
			subprocess.call(cmd, shell=True)


if __name__ == "__main__":

	if len(sys.argv) < 4:
		print 'Please specify three topics to subscribe (odom_wheel, odom_gps, imu)'
		sys.exit()

	topic_odom_wheel, topic_odom_gps, topic_imu = sys.argv[1], sys.argv[2], sys.argv[3]

	rospy.init_node('global_filter_wrapper', anonymous=True)

	node_name = rospy.get_name()
	config_file = rospy.get_param(node_name + '/config_file')

	global_filter_wrapper = GlobalFilterWrapper(topic_odom_wheel, topic_odom_gps, topic_imu, config_file)
	rospy.spin()
		
		
