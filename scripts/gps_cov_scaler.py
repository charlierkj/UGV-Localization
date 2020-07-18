#!/usr/bin/env python

import sys
import rospy

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class GPSCovScaler(object):
	# re-scale the GPS measurement covariance. 

	def __init__(self, topic_in, topic_out, scale, msr='position'):
		"""
		msr: 'position' (message type is Odometry) or 'heading' (message type is Imu)
		"""
		# configure subscriber and publisher
		if msr == 'position':
			self.sub = rospy.Subscriber(topic_in, Odometry, self.callback)
			self.pub = rospy.Publisher(topic_out, Odometry, queue_size=10)
		else:
			raise ValueError('msr attribute has to be "position"')
		
		self.scale = scale


	def callback(self, msg_in):
		msg_out = Odometry()
		msg_out.header = msg_in.header
		msg_out.pose.pose = msg_in.pose.pose
		for i in range(len(msg_in.pose.covariance)):
			msg_out.pose.covariance[i] = self.scale * msg_in.pose.covariance[i]

		self.pub.publish(msg_out)


if __name__ == "__main__":

	topic_in = '/odometry/gps'
	topic_out = '/odometry/gps_scaled'
	scale = 1000

	rospy.init_node('gps_cov_scaler', anonymous=True)
	gps_cov_scaler = GPSCovScaler(topic_in, topic_out, scale)
	rospy.spin()

	
