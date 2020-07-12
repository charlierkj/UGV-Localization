#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry

from tf import transformations
from tf2_msgs.msg import TFMessage


class CurvePlotter(object):

	def __init__(self):
	# hardcoded topics.
		self.start_time = -1
		self.record_nofilter = np.empty(shape=(0, 4)) # t, x, y, yaw
		self.record_filter = np.empty(shape=(0, 4))
		self.slip_angle = [] # slip angle for filtered odometry
		
		self.sub_nofilter = rospy.Subscriber('/odometry', Odometry, self.odom_callback)
		self.sub_filter = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
		self.sub_tf = rospy.Subscriber('/tf_static', TFMessage, self.tf_callback)

		self.tf_utm2map, self.tf_odom2utm, self.tf_map2odom = None, None, None


	def plot_curve_pose(self, path):
		fig, axs = plt.subplots(3)
		for i in range(3): # x, y, yaw
			axs[i].plot(self.record_nofilter[:, 0], self.record_nofilter[:, i+1], color='green', label='no filter')
			axs[i].plot(self.record_filter[:, 0], self.record_filter[:, i+1], color='red', label='filter')
		axs[0].set_ylabel('x')
		axs[1].set_ylabel('y')
		axs[2].set_ylabel('yaw')
		axs[2].set_xlabel('time')
		axs[0].legend()
		plt.savefig(path)
		plt.show()

	
	def plot_curve_slipangle(self, path):
		fig, axs = plt.subplots(2)
		# plot filtered yaw
		axs[0].plot(self.record_filter[:, 0], self.record_filter[:, 3])
		axs[0].set_ylabel('yaw')

		# plot slip angle
		axs[1].plot(self.record_filter[:, 0], self.slip_angle)
		axs[1].set_ylabel('slip angle')
		axs[1].set_xlabel('time')

		plt.savefig(path)
		plt.show()


	def odom_callback(self, msg_odom):
		time_curr = msg_odom.header.stamp.to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		print 'current timestamp: %f' % time_curr
		dt = time_curr - self.start_time
		p = msg_odom.pose.pose.position # position
		q = msg_odom.pose.pose.orientation # orientation (quaternion)
		euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # orientation (euler)

		if msg_odom.header.frame_id == '/world':
			yaw = euler[2] % (2 * np.pi) # restrict between [0, 2*PI]
			new_record = np.array([[dt, p.x, p.y, yaw]])
			self.record_nofilter = np.vstack((self.record_nofilter, new_record))
		elif msg_odom.header.frame_id == 'odom':
			trans_odom2baselink = transformations.translation_matrix([p.x, p.y, p.z])
			rot_odom2baselink = transformations.euler_matrix(euler[0], euler[1], euler[2])
			tf_odom2baselink = np.matmul(trans_odom2baselink, rot_odom2baselink)
			if self.tf_map2odom is not None:
				tf_map2baselink = np.matmul(self.tf_map2odom, tf_odom2baselink)
				x, y, _ = transformations.translation_from_matrix(tf_map2baselink)
				_, _, yaw = transformations.euler_from_matrix(tf_map2baselink)
				yaw = yaw % (2 * np.pi) # restrict between [0, 2*PI]
				new_record = np.array([[dt, x, y, yaw]])
				self.record_filter = np.vstack((self.record_filter, new_record))

				# slip angle
				vel_lin = msg_odom.twist.twist.linear # linear velocity
				slip_angle = - np.arctan2(vel_lin.y, np.abs(vel_lin.x))
				self.slip_angle.append(slip_angle)
		

	def tf_callback(self, msg_tfs):
		msg_tf = msg_tfs.transforms[0]
		p = msg_tf.transform.translation
		q = msg_tf.transform.rotation
		euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		if (msg_tf.header.frame_id == 'utm') and (msg_tf.child_frame_id == 'map') and (self.tf_utm2map is None):
			trans_utm2map = transformations.translation_matrix([p.x, p.y, p.z])
			rot_utm2map = transformations.euler_matrix(euler[0], euler[1], euler[2])
			self.tf_utm2map = np.matmul(trans_utm2map, rot_utm2map)
		elif (msg_tf.header.frame_id == 'odom') and (msg_tf.child_frame_id == 'utm') and (self.tf_odom2utm is None):
			trans_odom2utm = transformations.translation_matrix([p.x, p.y, p.z])
			rot_odom2utm = transformations.euler_matrix(euler[0], euler[1], euler[2])
			self.tf_odom2utm = np.matmul(trans_odom2utm, rot_odom2utm)

		if (self.tf_utm2map is not None) and (self.tf_odom2utm is not None):
			tf_odom2map = np.matmul(self.tf_odom2utm, self.tf_utm2map)
			self.tf_map2odom = transformations.inverse_matrix(tf_odom2map)
			self.sub_tf.unregister()
 

if __name__ == "__main__":

	rospy.init_node('plot_curve', anonymous=True)
	curve_plotter = CurvePlotter()
	rospy.spin()

	path = '/home/charlierkj/asco/src/ugv_localization/figs/test_03_slipangle.png'

	if rospy.is_shutdown():
		# curve_plotter.plot_curve_pose(path)
		curve_plotter.plot_curve_slipangle(path)
