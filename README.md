# qmul-trajectory-planner
ROS package that will automatically generate point-to-point cubic trajectories connecting pairs of randomly generated points.

A user should be able to install the package (i.e. download the .zip folder of the package, unzip it on their computer within
their catkin workspace and compile it), and after running a single launch file they should see different random trajectories
appearing on the rqt_plot GUI (i.e. a different set of trajectories every 20 seconds).

**LAUNCH FILE**
One ROS Launch file named cubic_traj_gen.launch should be created in the
launch folder, that will start the four Nodes and the rqt_plot GUI automatically.

**MESSAGES**
1. cubic_traj_params: contains 6 real values (float): p0, pf, v0, vf, t0, tf; i.e.
initial and final position, initial and final velocity, initial and final time.

2. cubic_traj_coeffs: contains 6 real values (float): a0, a1, a2, a3, t0, tf; i.e.
the four coefficients of a cubic polynomial trajectory, plus the initial and final time.

**SERVICE**
One ROS Service called compute_cubic_traj should be created in the srv folder,
which takes as input one full set of cubic trajectory parameters (initial and final
position, initial and final velocity, initial and final time), and returns the four
coefficients of the cubic polynomial trajectory (a0, a1, a2, a3).

**NODES**
1. points_generator.py: This node should generate different random values for initial and final
position, p0, pf, initial and final velocity, v0, vf, initial and final time, t0, tf, every 20
seconds, and publish them on a ROS Topic using the  cubic_traj_params message.
Positions should not exceed a maximum/minimum value P_MAX= +/- 10; velocities
should not exceed a mximum/minimum value V_MAX = +/- 10. For the time, t0
should always be 0, and tf should be tf = t0 + dt, with dt a random real number (i.e.
float) between 5 and 10.

2. cubic_traj_planner.py: This node should subscribe to the ROS Topic created by
Node 1, read the desired p0, pf, v0, vf, t0, tf published every 20 seconds, and
should compute the a0,a1,a2,a3 coefficients of the cubic polynonial trajectory of the
form p(t) = a0 + a1*t + a2*t^2 + a3*t^3 that best fit those requirements. To compute
the coefficients, the Node should call a "compute_cubic_traj" Service made
available by Node 3. Then, the Node should publish the a0,a1,a2,a3 coefficients
and t0,tf time parameters on a ROS Topic, using the cubic_traj_coeffs message.

3. compute_cubic_coeffs.py: This Node should made the "compute_cubic_traj"
Service available to any external node requesting it.

4.  plot_cubic_traj.py: This Node should subscribe to the ROS Topic created by
Node 2, read the a0,a1,a2,a3 coefficients and t0,tf time parameters published every
20 seconds, and publish three separate ROS topics: position trajectory, velocity
trajectory and acceleration trajectory. These trajectories should be then visualized
with the rqt_plot GUI, on the same plot, with different colors. The three trajectories
should appear on the GUI at the same time, and last for tf seconds.