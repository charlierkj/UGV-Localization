#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry

traj_waypts= np.empty(shape=(0, 2))

def plot_traj(topic_odom):
    rospy.init_node('plot_traj', anonymous=True)
    rospy.Subscriber(topic_odom, Odometry, odom_callback)
    rospy.spin()

def odom_callback(msg_odom):
    # save trajectory waypoints
    waypt_2d = np.array([msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y])
    global traj_waypts 
    traj_waypts = np.vstack((traj_waypts, waypt_2d))
    if traj_waypts.shape[0] > 1:
	plt.plot(traj_waypts[:, 0], traj_waypts[:, 1], color='blue')
	plt.pause(0.01)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print 'Please specify odometry topic'
        sys.exit()

    topic_odom = sys.argv[1]
    plot_traj(topic_odom)
