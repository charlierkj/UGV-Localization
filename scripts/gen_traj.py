#!/usr/bin/env python

import os, sys
import rospy
import rospkg

import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped

from traj_opt import TrajOptimizer


class WaypointsSaver(object):

	def __init__(self, topic, max_num=30):
		self.max_num = max_num

		self.sub = rospy.Subscriber(topic, PointStamped, self.callback)
		self.waypoints = np.empty(shape=(0, 2)) # x, y

		self.click_start = False
		self.last_waypoint = None
		print("Please double click on the map to start")

	def callback(self, msg_wp):
		point = np.array([msg_wp.point.x, msg_wp.point.y])
		if self.last_waypoint is not None:
			diff = np.linalg.norm(point - self.last_waypoint)
			if diff <= 0.1:
				self.click_start = not self.click_start
			if self.click_start:
				if self.waypoints.shape[0] < self.max_num:
					self.waypoints = np.vstack((self.waypoints, point.reshape(1, 2)))
		self.last_waypoint = point


	def save_wps(self, path):
		np.save(path, self.waypoints)


	def get_waypoints(self):
		return self.waypoints


class TrajGenerator(WaypointsSaver):

	def __init__(self, topic_in, topic_out, max_num=30, vel=1, accel=1, save_npy=False, save_traj_file=None, save_plot_file=None):
		super(TrajGenerator, self).__init__(topic_in, max_num)

		self.traj_optimizer = TrajOptimizer(topic_out, vel=vel, accel=accel)
		self.save_npy = save_npy
		self.save_traj_file = save_traj_file
		self.save_plot_file = save_plot_file


	def reset(self):
		self.waypoints = np.empty(shape=(0, 2))
		self.last_waypoint = None


	def callback(self, msg_wp):
		point = np.array([msg_wp.point.x, msg_wp.point.y])
		if self.last_waypoint is not None:
			diff = np.linalg.norm(point - self.last_waypoint)
			if diff <= 0.1:
				self.click_start = not self.click_start
				if self.click_start:
					print("Start recording clicked points.")
				if not self.click_start:
					print("End recording clicked points.")
					if self.save_npy:
						self.save_wps(os.path.join(rospkg.RosPack.get_path("ugv_localization"), "records", "wps.npy"))
					self.gen_traj()
					self.reset()

			if self.click_start:
				if self.waypoints.shape[0] < self.max_num:
					self.waypoints = np.vstack((self.waypoints, point.reshape(1, 2)))
		self.last_waypoint = point


	def save_wps(self, path):
		np.save(path, self.waypoints)


	def gen_traj(self):
		print("Generating reference trajectory ...")
		self.traj_optimizer.set_wps(self.waypoints)
		self.traj_optimizer.estimate_segment_times()
		self.traj_optimizer.generate_traj()
		fig = self.traj_optimizer.plot_traj(0.1)
		msg = self.traj_optimizer.publish_reftraj()
		print("Done. Message published to topic: %s" % self.traj_optimizer.topic_out)
		if self.save_traj_file != "None":
			self.traj_optimizer.save_reftraj(msg, self.save_traj_file)
		if self.save_plot_file != "None":
			self.traj_optimizer.save_plot(fig, self.save_plot_file)


if __name__ == "__main__":

	# rospy.init_node('click_waypoints', anonymous=True)

	# topic = "mapviz/clicked_point"
	# npy_path = "/home/charlierkj/asco/src/ugv_localization/records/wps.npy"

	# waypoints_saver = WaypointsSaver(topic)
	# rospy.spin()

	# if rospy.is_shutdown():
	#	waypoints_saver.save_wps(npy_path)

	rospy.init_node('gen_traj', anonymous=True)

	node_name = rospy.get_name()
	topic_in = rospy.get_param(node_name + '/topic_in')
	topic_out = rospy.get_param(node_name + '/topic_out')
	max_num = int(rospy.get_param(node_name + '/maxnum_clicked_waypoints'))
	vel = float(rospy.get_param(node_name + '/velocity'))
	accel = float(rospy.get_param(node_name + '/acceleration'))
	save_traj_file = rospy.get_param(node_name + '/save_traj_file')
	save_plot_file = rospy.get_param(node_name + '/save_plot_file')

	traj_generator = TrajGenerator(topic_in, topic_out, max_num=max_num, save_npy=False, save_traj_file=save_traj_file)
	rospy.spin()


