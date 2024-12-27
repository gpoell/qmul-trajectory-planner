#!/usr/bin/env python3

import rospy
from ar_week8_test.msg import cubic_traj_params, cubic_traj_coeffs
from ar_week8_test.srv import compute_cubic_traj

"""
NODE: Cubic Trajectory Planner

This node listens for initial and final parameters (time, position, velocity),
calculates the cubic polynomial coefficients, and publishes them for plotting.
"""


def callback(msg):

    rospy.wait_for_service('compute_cubic_traj')

    # Execute service and return Srv response
    try:
        proxy = rospy.ServiceProxy('compute_cubic_traj', compute_cubic_traj)
        response = proxy(t0=msg.t0, tf=msg.tf, p0=msg.p0, pf=msg.pf, v0=msg.v0, vf=msg.vf)

        # Generate new message to publish
        msg_coeffs = cubic_traj_coeffs()
        msg_coeffs.t0 = msg.t0
        msg_coeffs.tf = msg.tf
        msg_coeffs.a0 = response.a0
        msg_coeffs.a1 = response.a1
        msg_coeffs.a2 = response.a2
        msg_coeffs.a3 = response.a3

        # Publish the Coefficients
        pub = rospy.Publisher("cubic_traj_coeffs", cubic_traj_coeffs, queue_size=10)
        pub.publish(msg_coeffs)


    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def cubic_traj_planner():
    rospy.init_node('cubic_traj_planner', anonymous=True)
    rospy.Subscriber("cubic_traj_params", cubic_traj_params, callback)
    rospy.spin()

if __name__ == '__main__':
    cubic_traj_planner()
