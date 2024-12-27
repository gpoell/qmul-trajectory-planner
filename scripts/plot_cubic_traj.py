#!/usr/bin/env python3

import rospy
import numpy as np
from ar_week8_test.msg import cubic_traj_coeffs
from std_msgs.msg import Float32


"""
NODE: Plot Cubic Trajectory

This service actively listens for trajectory coefficients to calculate the cubic
polynomial trajectory and publish the position, velocity, and acceleration
values to be plotted by rqt_plot

    t0: initial time
    tf: final time
    a0: first coefficient
    a1: second coefficient
    a2: third coefficient
    a3: fourth coefficient
"""


# Callback function used for calculating the cubic trajectory for position, velocity, and acceleration
def callback(msg):
    # Unpack trajectory coefficient message
    a0, a1, a2, a3, t0, tf = msg.a0, msg.a1, msg.a2, msg.a3, msg.t0, msg.tf

    # Generate time increments to solve for trajectory values
    time = np.linspace(t0, tf, 200)

    # Publishers and topics for position, velocity, and acceleration
    pub_pos = rospy.Publisher('traj/position', Float32, queue_size=10)
    pub_vel = rospy.Publisher('traj/velocity', Float32, queue_size=10)
    pub_acc = rospy.Publisher('traj/acceleration', Float32, queue_size=10)

    # Calculate Cubic Trajectory and publish values over time increment
    for t in time:
        pos = a0 + (a1*t) + (a2*t**2) + (a3*t**3)
        vel = a1 + (2*a2*t) + (3*a3*t**2)
        acc = (2*a2) + (6*a3*t)
        # Throttle the rate to smoothly publish points
        rospy.Rate(100).sleep()
        # Publish values to topics for rqt_plot
        pub_pos.publish(pos)
        pub_vel.publish(vel)
        pub_acc.publish(acc)

if __name__ == '__main__':
    rospy.init_node('plot_cubic_traj')
    s = rospy.Subscriber('cubic_traj_coeffs', cubic_traj_coeffs, callback)
    rospy.spin()
