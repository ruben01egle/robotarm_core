# robotarm_trajectory_smoothing

Jerk-limited smoothing of whole joint trajectories for the Pilz motions of `robotarm_motion`.
Node `trajectory_smoother` (started by `robotarm_moveit_config/launch/move_group.launch.py`),
service `/smooth_trajectory` (`robotarm_interface/srv/SmoothTrajectory`).

Pilz trajectories have trapezoidal profiles, and in LIN motions and blends their accelerations are
finite differences of IK samples that jump from point to point. joint_trajectory_controller passes
them to ff_controller as feedforward torque, so the arm gets torque steps. The smoother filters
the joint positions over time with two moving averages (0.1 s, then the window the jerk limits of
`joint_limits.yaml` need) and computes velocity and acceleration exactly from the result:

- jerk within `max_jerk`, also between the points (quintic splines of the controller)
- never faster and never harder accelerating than the plan
- start, end and every stop (all joints at rest, a step without `blend`) exact: the trajectory is
  split there and every piece is filtered on its own (about 0.2–0.6 s more per stop)
- blends slightly rounded (well below 1 mm at 20 % speed)

Details in `include/robotarm_trajectory_smoothing/smoothing.hpp`. The algorithm
(`src/smoothing.cpp`) has no ROS node or robot model; the node (`src/trajectory_smoother.cpp`)
only reads the jerk limits from the robot model.

## Tests

```bash
colcon build --packages-select robotarm_trajectory_smoothing
colcon test --packages-select robotarm_trajectory_smoothing && colcon test-result --verbose
```

`test/test_smoothing.cpp` checks, on rest-to-rest motions sampled like Pilz: exact start, end and
stops, the jerk limit between the points, no overshoot of velocity and acceleration, consistent
derivatives, timing, the window from the acceleration range, one window for all joints (straight
lines in joint space stay straight), that input accelerations are ignored, and invalid input.

The effect on real programs: `robotarm_motion/scripts/compare_smoothing.py`.
