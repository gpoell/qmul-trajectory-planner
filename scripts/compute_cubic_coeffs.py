#!/usr/bin/env python

import rospy
import numpy as np
from ar_week8_test.srv import compute_cubic_traj, compute_cubic_trajResponse


"""
NODE: Service

Cubic Coefficient Service

This service calculates the cubic polynomial coefficients based on the trajectory parameters
provided by the cubic trajectory planner.

    t0: initial time
    tf: final time
    p0: initial position
    pf: final position
    v0: initial velocity
    vf: final velocity
"""

def compute_cubic_trajectory(req):
    # Unpack the message for ease of reference
    t0, tf, p0, pf, v0, vf = req.t0, req.tf, req.p0, req.pf, req.v0, req.vf

    # Calculate and return the coefficients using system of equations
    a = np.array([
        [1, t0, t0**2, t0**3],
        [0, 1, 2*t0, 3*t0**2],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])

    b = np.array([p0, v0, pf, vf])

    coeffs = np.linalg.solve(a, b)

    return compute_cubic_trajResponse(coeffs[0], coeffs[1], coeffs[2], coeffs[3])


if __name__ == "__main__":
    rospy.init_node('compute_cubic_traj_server')
    s = rospy.Service('compute_cubic_traj', compute_cubic_traj, compute_cubic_trajectory)
    print("Ready to compute cubic coefficients...")
    rospy.spin()