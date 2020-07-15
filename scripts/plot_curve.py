#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from tf import transformations
from tf2_msgs.msg import TFMessage


class CurvePlotter(object):

	def __init__(self, mode='global'):
	# hardcoded topics.
		self.mode = mode

		self.start_time = -1
		self.record_nofilter = np.empty(shape=(0, 4)) # t, x, y, yaw
		self.record_filter = np.empty(shape=(0, 4))
		self.slip_angle = [] # slip angle for filtered odometry

		self.gps_msr = np.empty(shape=(0, 5)) # t, lat, lat_cov, lon, lon_cov
		self.heading_msr = np.empty(shape=(0, 3)) # t, yaw, yaw_cov
		
		self.sub_nofilter = rospy.Subscriber('/odometry', Odometry, self.odom_callback)

		if self.mode == 'local':
			self.sub_tf = rospy.Subscriber('/tf_static', TFMessage, self.tf_callback)

		self.sub_filter = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

		#self.sub_gps = rospy.Subscriber('/rtk_ublox/fix', NavSatFix, self.gps_callback)
		#self.sub_heading = rospy.Subscriber('/imu/combined', Imu, self.heading_callback)

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


	def plot_covariance_gps(self, path):
		fig, axs = plt.subplots(2, 2)

		# plot lat with covariance
		axs[0, 0].plot(self.gps_msr[:, 0], self.gps_msr[:, 1])
		axs[0, 0].set_ylabel('latitude')
		axs[1, 0].plot(self.gps_msr[:, 0], self.gps_msr[:, 2])
		axs[1, 0].set_ylabel('latitude covariance')
		axs[1, 0].set_xlabel('time')

		# plot lon with covariance
		axs[0, 1].plot(self.gps_msr[:, 0], self.gps_msr[:, 3])
		axs[0, 1].set_ylabel('longitude')
		axs[1, 1].plot(self.gps_msr[:, 0], self.gps_msr[:, 4])
		axs[1, 1].set_ylabel('longitude covariance')
		axs[1, 1].set_xlabel('time')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def plot_covariance_heading(self, path):
		fig, axs = plt.subplots(2)

		# plot heading measurements with covariance
		axs[0].plot(self.heading_msr[:, 0], self.heading_msr[:, 1])
		axs[0].set_ylabel('heading measurement')
		axs[1].plot(self.heading_msr[:, 0], self.heading_msr[:, 2])
		axs[1].set_ylabel('measurement covariance')
		axs[1].set_xlabel('time')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def odom_callback(self, msg_odom):
		time_curr = msg_odom.header.stamp.to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		# print 'current timestamp: %f' % time_curr
		dt = time_curr - self.start_time
		p = msg_odom.pose.pose.position # position
		q = msg_odom.pose.pose.orientation # orientation (quaternion)
		euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # orientation (euler)

		if msg_odom.header.frame_id == '/world':
			yaw = euler[2] % (2 * np.pi) # restrict between [0, 2*PI]
			new_record = np.array([[dt, p.x, p.y, yaw]])
			self.record_nofilter = np.vstack((self.record_nofilter, new_record))

		elif msg_odom.header.frame_id == 'odom' or msg_odom.header.frame_id == 'map':
			if self.mode == 'local':
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
			
			elif self.mode == 'global':
				yaw = euler[2] % (2 * np.pi)
				new_record = np.array([[dt, p.x, p.y, yaw]])
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


	def gps_callback(self, msg_gps):
		time_curr = msg_gps.header.stamp.to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		new_record = np.array([[dt, msg_gps.latitude, msg_gps.position_covariance[0], msg_gps.longitude, msg_gps.position_covariance[4]]])
		self.gps_msr = np.vstack((self.gps_msr, new_record))


	def heading_callback(self, msg_heading):
		time_curr = msg_heading.header.stamp.to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		
		q = msg_heading.orientation
		_, _, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])	
		new_record = np.array([[dt, yaw, msg_heading.orientation_covariance[8]]])
		self.heading_msr = np.vstack((self.heading_msr, new_record))

		

if __name__ == "__main__":

	rospy.init_node('plot_curve', anonymous=True)

	node_name = rospy.get_name()
	mode = rospy.get_param(node_name + '/mode')
	data = rospy.get_param(node_name + '/data')

	curve_plotter = CurvePlotter(mode)
	rospy.spin()

	path_pose = '/home/charlierkj/asco/src/ugv_localization/figs/%s_pose.png' % data
	path_slipangle = '/home/charlierkj/asco/src/ugv_localization/figs/%s_slipangle.png' % data
	path_gps = '/home/charlierkj/asco/src/ugv_localization/figs/%s_gps_msr.png' % data
	path_heading = '/home/charlierkj/asco/src/ugv_localization/figs/%s_heading_msr.png' % data

	if rospy.is_shutdown():
		curve_plotter.plot_curve_pose(path_pose)
		curve_plotter.plot_curve_slipangle(path_slipangle)
		#curve_plotter.plot_covariance_gps(path_gps)
		#curve_plotter.plot_covariance_heading(path_heading)
