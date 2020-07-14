#!/usr/bin/env python

import sys
import rospy

import numpy as np

import message_filters

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from imu_3dm_gx4.msg import FilterOutput

from tf import transformations


class ImuCombine(object):
	# combine separate imu topics to publish single imu topic
	# and convert it to conventional configuration:
	# 	orientations measured in east-X, north-Y, up-Z world frame,
	#	local frame transformed from 'imu_3dm' to 'base_link' 

	def __init__(self, topic_in_1, topic_in_2, topic_out):
		# configure subscriber and publisher
		self.sub_imu = message_filters.Subscriber(topic_in_1, Imu)
		self.sub_filter = message_filters.Subscriber(topic_in_2, FilterOutput)
		self.pub_imu = rospy.Publisher(topic_out, Imu, queue_size=10)

		# time synchronizer
		self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_imu, self.sub_filter], 10, 0.01)
		self.ts.registerCallback(self.callback)

		# transformations
		self.rot_enu2ned = transformations.quaternion_from_euler(np.pi, 0, np.pi / 2, 'sxyz') # static axis
		self.rot_baselink2imu = transformations.quaternion_from_euler(np.pi, 0, 0)
		self.rot_imu2baselink = transformations.quaternion_inverse(self.rot_baselink2imu)


	def callback(self, msg_imu, msg_filter):
		msg_out = Imu()

		msg_out.header = msg_imu.header
		msg_out.header.frame_id = "base_link"
		msg_out.angular_velocity = self.convert_angular_vel(msg_imu.angular_velocity)
		msg_out.angular_velocity_covariance = msg_imu.angular_velocity_covariance
		msg_out.linear_acceleration = msg_imu.linear_acceleration
		msg_out.linear_acceleration_covariance = msg_imu.linear_acceleration_covariance

		msg_out.orientation = self.convert_orientation(msg_filter.orientation)
		msg_out.orientation_covariance = msg_filter.orientation_covariance

		self.pub_imu.publish(msg_out)


	def convert_angular_vel(self, msg_in):
		q_in = transformations.quaternion_from_euler(msg_in.x, msg_in.y, msg_in.z)
		q_out = transformations.quaternion_multiply(self.rot_baselink2imu, q_in)
		v_out = transformations.euler_from_quaternion(q_out)
		msg_out = Vector3()
		msg_out.x = v_out[0]
		msg_out.y = v_out[1]
		msg_out.z = v_out[2]
		return msg_out


	def convert_orientation(self, msg_in):
		# convert from (north-X, east-Y, down-Z) reference frame to (east-X, north-Y, up-Z)
		rot_ned2imu = np.array([msg_in.x, msg_in.y, msg_in.z, msg_in.w])
		q_out = transformations.quaternion_multiply(transformations.quaternion_multiply(self.rot_enu2ned, rot_ned2imu), self.rot_imu2baselink)
		msg_out = Quaternion()
		msg_out.x = q_out[0]
		msg_out.y = q_out[1]
		msg_out.z = q_out[2]
		msg_out.w = q_out[3]
		return msg_out


class ImuRefine(ImuCombine):
	# refine single imu measurements from from 'imu_3dm' frame to 'base_link' frame. 

	def __init__(self, topic_in, topic_out):
		# configure subscriber and publisher
		self.sub_imu = rospy.Subscriber(topic_in, Imu, self.callback)
		self.pub_imu = rospy.Publisher(topic_out, Imu, queue_size=10)

		# transformations
		self.rot_baselink2imu = transformations.quaternion_from_euler(np.pi, 0, 0)


	def callback(self, msg_in):
		msg_out = msg_in

		msg_out.header.frame_id = "base_link"
		msg_out.header.stamp = rospy.Time.now()
		msg_out.angular_velocity = self.convert_angular_vel(msg_in.angular_velocity)
		msg_out.angular_velocity_covariance = msg_in.angular_velocity_covariance

		self.pub_imu.publish(msg_out)


	def convert_angular_vel(self, msg_in):
		return super(ImuRefine, self).convert_angular_vel(msg_in)
		

if __name__ == "__main__":

	rospy.init_node('imu_combine', anonymous=True)

	node_name = rospy.get_name()
	mode = rospy.get_param(node_name + '/mode')

	if mode == 'combine':
		# hardcoded topics
		topic_in_1 = 'imu_3dm/imu'
		topic_in_2 = 'imu_3dm/filter'
		topic_out = 'imu/combined'

		imu_combine = ImuCombine(topic_in_1, topic_in_2, topic_out)

	elif mode == 'refine':
		topic_in = 'imu_3dm/imu'
		topic_out = 'imu/refined'
	
		imu_refine = ImuRefine(topic_in, topic_out)

	rospy.spin()

	
