#!/usr/bin/env python

import os, sys
import rospy
import rospkg

import numpy as np

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import InteractiveMarkerFeedback

from traj_opt import TrajOptimizer
from waypoints_marker import WaypointsMarkerManager


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

	def __init__(self, topic_in, topic_out, max_num=30, vel=1, accel=1, save_npy=False, save_traj_file=None, save_plot_file=None, refine=False):
		super(TrajGenerator, self).__init__(topic_in, max_num)

		self.traj_optimizer = TrajOptimizer(topic_out, vel=vel, accel=accel)
		self.save_npy = save_npy
		self.save_traj_file = save_traj_file
		self.save_plot_file = save_plot_file

		self.refine = refine

		if self.refine:
			self.sub_refine = rospy.Subscriber("waypoints_marker/feedback", InteractiveMarkerFeedback, self.callback_refine)


	def construct_marker_server(self):
		if self.refine:
			if hasattr(self, "marker_server"):
				self.marker_server.reset()
				print("InteractiveMarker server reset.")
			else:
				self.marker_server = WaypointsMarkerManager()
				print("InteractiveMarker server constructed.")


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
					self.construct_marker_server()
					print("Start recording clicked points.")
				if not self.click_start:
					print("End recording clicked points.")
					if self.save_npy:
						self.save_wps(os.path.join(rospkg.RosPack.get_path("ugv_localization"), "records", "wps.npy"))
					print("Generating reference trajectory ...")
					self.gen_traj()
					self.reset()

			if self.click_start:
				if self.waypoints.shape[0] < self.max_num:
					self.waypoints = np.vstack((self.waypoints, point.reshape(1, 2)))

					if hasattr(self, "marker_server"):
						self.marker_server.add_waypoints(point)
		self.last_waypoint = point


	def callback_refine(self, marker_feedback):
		if marker_feedback.event_type == 1:
			self.waypoints = self.marker_server.get_waypoints()
		if (marker_feedback.control_name == "start_refine") and (marker_feedback.event_type == 3):
			self.refine_traj()


	def save_wps(self, path):
		np.save(path, self.waypoints)


	def gen_traj(self):
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


	def refine_traj(self):
		print("Refining reference trajectory ...")
		self.waypoints = self.marker_server.get_waypoints()
		self.gen_traj()


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
	refine = rospy.get_param(node_name + '/enable_refine')

	traj_generator = TrajGenerator(topic_in, topic_out, max_num=max_num, vel=vel, accel=accel, save_npy=False, save_traj_file=save_traj_file, save_plot_file=save_traj_file, refine=refine)
	rospy.spin()


