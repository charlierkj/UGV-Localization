#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize

MAGIC_FABIAN_CONST = 6.5


class TrajOptimizer(object):

	def __init__(self, loop=False):
		self.polys_x = None
		self.polys_y = None
		self.time = None

		self.loop = loop


	def load_npy(self, path):
		self.waypoints = np.load(path)
		self.num_segments = self.waypoints.shape[0] - 1
		if self.loop:
			self.waypoints = np.vstack((self.waypoints, np.zeros(shape=(1, 2))))
			self.num_segments += 1
		self.durations = np.zeros(shape=(self.num_segments,))
		self.time = np.sum(self.durations)


	def estimate_segment_times(self):
		v_max = 2.0 # hardcoded
		a_max = 2.0
		for seg_i in range(self.num_segments):
			start = self.waypoints[seg_i, :]
			end = self.waypoints[seg_i + 1, :]
			distance = np.linalg.norm(end - start)
			t = distance / v_max * 2 * (1.0 + MAGIC_FABIAN_CONST * v_max / a_max * np.exp(- distance / v_max * 2))
			self.durations[seg_i] = t
		print(self.durations)


	def objective_func(self, polys):
		# minimize integral of snap squared
		cost = 0
		for seg_i in range(self.num_segments):
			poly = polys[seg_i * 8 : (seg_i + 1) * 8]
			snap = np.polyder(poly, 4)
			for t in np.arange(0, self.durations[seg_i], 0.01):
				cost_i = np.polyval(snap, t) ** 2 * 0.01
				cost += cost
			#snap_der = np.polyder(poly, 5)
			# cost_seg = (np.polyval(snap, self.durations[seg_i]) ** 3 / np.polyval(snap_der, self.durations[seg_i]) - np.polyval(snap, 0) ** 3 / np.polyval(snap_der, 0)) / 3
			#cost_seg = np.polyval(snap, self.durations[seg_i]) ** 3
			#cost += cost_seg
		return cost


	def constraint_position_start(self, polys, axis, seg_i):
		poly = polys[seg_i * 8 : (seg_i + 1) * 8]
		value = np.polyval(poly, 0)
		start = self.waypoints[seg_i, axis]
		return value - start


	def constraint_position_end(self, polys, axis, seg_i):
		poly = polys[seg_i * 8 : (seg_i + 1) * 8]
		value = np.polyval(poly, self.durations[seg_i])
		end = self.waypoints[seg_i + 1, axis]
		#if seg_i == self.num_segments - 1:
		#	end = self.waypoints[-1, axis]
		#else:
		#	poly_next = polys[(seg_i + 1) * 8 : (seg_i + 2) * 8]
		#	end = np.polyval(poly_next, 0)
		return value - end


	def constraint_der(self, polys, axis, seg_i):
		if seg_i == 0:
			return 0
		else:
			poly_last = polys[(seg_i - 1) * 8 : seg_i * 8]
			poly = polys[seg_i * 8: (seg_i + 1) * 8]
			der_last = np.polyder(poly_last, 1)
			der = np.polyder(poly, 1)
			end_der = np.polyval(der_last, self.durations[seg_i - 1])
			start_der = np.polyval(der, 0)
			return start_der - end_der


	def generate_polys(self, axis):
		polys_init = np.zeros(self.num_segments * 8)
		constraints = []
		for seg_i in range(self.num_segments):
			constraints.append({'type': 'eq', 'fun': self.constraint_position_start, 'args': (axis, seg_i)})
			constraints.append({'type': 'eq', 'fun': self.constraint_position_end, 'args': (axis, seg_i)})
			# constraints.append({'type': 'eq', 'fun': self.constraint_der, 'args': (axis, seg_i)})

		sol = scipy.optimize.minimize(self.objective_func, polys_init, method="SLSQP", options={"maxiter":100}, constraints=constraints)
		return sol.x


	def generate_traj(self):
		self.polys_x = self.generate_polys(0)
		self.polys_y = self.generate_polys(1)
		print(self.polys_x)
		print(self.polys_y)


	def plot_traj(self):
		for seg_i in range(self.num_segments):
			for t in np.arange(0, self.durations[seg_i], 0.05):
				x = np.polyval(self.polys_x[seg_i * 8 : (seg_i + 1) * 8], t)
				y = np.polyval(self.polys_y[seg_i * 8 : (seg_i + 1) * 8], t)
				if t == 0:
					print(x, y)
				plt.scatter(x, y)
		plt.show()
			


if __name__ == "__main__":

	traj_opt = TrajOptimizer()
	npy_path = "/home/charlierkj/asco/src/ugv_localization/records/wps.npy"
	traj_opt.load_npy(npy_path)
	traj_opt.estimate_segment_times()
	traj_opt.generate_traj()
	traj_opt.plot_traj()



