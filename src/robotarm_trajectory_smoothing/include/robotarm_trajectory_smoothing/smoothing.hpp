// Jerk-limited smoothing of whole joint trajectories (moving-average / FIR jerk limitation).
//
// Pilz trajectories have trapezoidal profiles, and in blends and LIN motions the accelerations are
// finite differences of IK samples: they jump from point to point. joint_trajectory_controller
// passes them to ff_controller as feedforward torque, so the arm gets torque steps. MoveIt's own
// Ruckig smoothing (AddRuckigTrajectorySmoothing) cannot fix such trajectories: it has to reach
// the acceleration of every waypoint exactly, stretches the timing to its maximum and gives up.
//
// Instead the trajectory is filtered with moving averages, as in industrial controllers. A moving
// average over a window T gives
//   v_s(t) = (q(t) - q(t - T)) / T,  a_s(t) = (v(t) - v(t - T)) / T,  j_s(t) = (a(t) - a(t - T)) / T
// so |j_s| <= (max a - min a) / T per joint. Two filters run one after the other:
//   1. PREFILTER_WINDOW (the 0.1 s sampling of Pilz) removes the point-to-point spikes of the
//      accelerations, which would otherwise ask for a long window in step 2,
//   2. a window T from the acceleration range left after step 1: the smallest that keeps every
//      joint within its jerk limit.
// Velocities and accelerations are averages and never exceed those of the input. All joints use
// the same windows, so the path stays close to the input's; blends are rounded a little more.
// Positions, velocities and accelerations of the result are exactly consistent, so the quintic
// splines that joint_trajectory_controller lays through the points keep the jerk limit between
// them too.
//
// Stops are kept exact: the trajectory is split at every inner point where all joints are at rest
// (a step without blending; Pilz plans exactly zero velocity there), and each piece is filtered on
// its own, with its own windows. Every piece starts and ends at rest exactly on its stop position;
// each piece gets longer by its two windows.
//
// The input is interpolated with cubic Hermite splines (positions and velocities); its
// accelerations are not used. It has to start and end at rest.

#pragma once

#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace robotarm_trajectory_smoothing
{
constexpr double GRID_DT = 0.001;          // [s] integration grid
constexpr double OUTPUT_DT = 0.02;         // [s] spacing of the output points
constexpr double PREFILTER_WINDOW = 0.1;   // [s] first filter
constexpr double MIN_WINDOW = 0.01;        // [s] shortest second filter
constexpr double STOP_VELOCITY = 1e-6;     // [rad/s] all joints below at an inner point: a stop
constexpr double END_VELOCITY = 1e-3;      // [rad/s] tolerance for "starts and ends at rest"

struct SmoothingInfo
{
  size_t pieces = 0;          // pieces between stops
  double min_window = 0.0;    // [s] shortest / longest second filter window of the pieces
  double max_window = 0.0;
  double duration_in = 0.0;   // [s]
  double duration_out = 0.0;  // [s]
};

// Smooth `in` (positions and velocities of all joints at every point, times increasing, at rest at
// both ends) with the jerk limits max_jerk [rad/s^3] (one per joint, in the order of
// in.joint_names). On success, `out` has positions, velocities and accelerations every OUTPUT_DT
// and at every stop and the end, and `info` (if given) describes the result. On failure, returns
// false and sets `message`.
bool smoothTrajectory(const trajectory_msgs::msg::JointTrajectory& in, const std::vector<double>& max_jerk,
                      trajectory_msgs::msg::JointTrajectory& out, std::string& message,
                      SmoothingInfo* info = nullptr);

// Seconds of a builtin_interfaces/Duration.
double seconds(const builtin_interfaces::msg::Duration& d);

}  // namespace robotarm_trajectory_smoothing
