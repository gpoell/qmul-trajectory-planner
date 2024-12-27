#!/usr/bin/env python3

import rospy
from ar_week8_test.msg import cubic_traj_params
import random


"""
NODE: Cubic Polynomial Points Generator

This node publishes initial conditions for calculating the cubic polynomial trajectory
"""

# Create node, publisher, and publishing rate of 20 seconds
rospy.init_node('points_generator')
pub = rospy.Publisher('cubic_traj_params', cubic_traj_params, queue_size=10)
rate = rospy.Rate(0.05)

rospy.loginfo("Publishing trajectory points...")

while not rospy.is_shutdown():
    # Generate and publish initial conditions specified in assignment
    dt = random.uniform(5, 10)
    msg = cubic_traj_params()
    msg.p0 = random.uniform(-10, 10)
    msg.pf = random.uniform(-10, 10)
    msg.v0 = random.uniform(-10, 10)
    msg.vf = random.uniform(-10, 10)
    msg.t0 = 0
    msg.tf = msg.t0 + dt

    pub.publish(msg)
    rate.sleep()