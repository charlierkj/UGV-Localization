#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

traj_waypts= np.empty(shape=(0, 2))

def plot_traj(topic, sensor='odom'):
    rospy.init_node('plot_traj', anonymous=True)
    if sensor == 'odom':
    	rospy.Subscriber(topic, Odometry, odom_callback)
    elif sensor == 'gps':
		rospy.Subscriber(topic, NavSatFix, gps_callback)
    elif sensor == 'imu':
        rospy.Subscriber(topic, Imu, imu_callback)
    rospy.spin()

def odom_callback(msg_odom):
    # save trajectory waypoints
    waypt_2d = np.array([msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y])
    global traj_waypts 
    traj_waypts = np.vstack((traj_waypts, waypt_2d))
    if traj_waypts.shape[0] > 1:
        plt.plot(traj_waypts[:, 0], traj_waypts[:, 1], color='blue')
        plt.pause(0.01)

def gps_callback(msg_gps):
    # save trajectory waypoints
    waypt_2d = np.array([msg_gps.longitude, msg_gps.latitude])
    global traj_waypts 
    traj_waypts = np.vstack((traj_waypts, waypt_2d))
    if traj_waypts.shape[0] > 1:
	    plt.plot(traj_waypts[:, 0], traj_waypts[:, 1], color='blue')
	    plt.pause(0.01)

def imu_callback(msg_imu):
    # save trajectory waypoints
    dt = 0.01
    global traj_waypts 
    if traj_waypts.shape[0] == 0:
        waypt_2d = np.zeros(shape=(1, 2))
        traj_waypts = np.vstack((traj_waypts, waypt_2d))
        global vel_last, angle_last
        vel_last, angle_last = 0, 0

    dist = (msg_imu.linear_acceleration.x / 2 * dt + vel_last) * dt
    vel_last = vel_last + msg_imu.linear_acceleration.x * dt
    angle_last = angle_last - msg_imu.angular_velocity.z * dt
    waypt_2d = traj_waypts[-1, :] + dist * np.array([np.cos(angle_last), np.sin(angle_last)])
    traj_waypts = np.vstack((traj_waypts, waypt_2d))

    if traj_waypts.shape[0] > 16370:
        plt.plot(traj_waypts[:, 0], traj_waypts[:, 1], color='blue')
    #plt.clf()
    #plt.plot([0, np.cos(angle_last)], [0, np.sin(angle_last)])
    #plt.xlim((-1, 1))
    #plt.ylim((-1, 1))
        plt.axis('equal')
        plt.pause(0.01)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print 'Please specify odometry topic'
        sys.exit()

    topic = sys.argv[1]
    plot_traj(topic, sensor='imu')
