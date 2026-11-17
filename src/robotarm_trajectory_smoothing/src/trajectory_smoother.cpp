// Jerk-limited smoothing of whole joint trajectories, service /smooth_trajectory
// (robotarm_interface/srv/SmoothTrajectory).
//
// Pilz trajectories have trapezoidal profiles, and in blends and LIN motions the accelerations are
// finite differences of IK samples: they jump from point to point. joint_trajectory_controller
// passes them to ff_controller as feedforward torque, so the arm gets torque steps. MoveIt's own
// Ruckig smoothing (AddRuckigTrajectorySmoothing) cannot fix such trajectories: it has to reach
// the acceleration of every waypoint exactly, stretches the timing to its maximum and gives up.
//
// Instead the trajectory is filtered with moving averages (FIR jerk limitation, as in industrial
// controllers). A moving average over a window T gives
//   v_s(t) = (q(t) - q(t - T)) / T,  a_s(t) = (v(t) - v(t - T)) / T,  j_s(t) = (a(t) - a(t - T)) / T
// so |j_s| <= (max a - min a) / T per joint. Two filters run one after the other:
//   1. PREFILTER_WINDOW (the 0.1 s sampling of Pilz) removes the point-to-point spikes of the
//      accelerations, which would otherwise ask for a long window in step 2,
//   2. a window T from the acceleration range left after step 1: the smallest that keeps every
//      joint within its max_jerk (joint_limits.yaml).
// Velocities and accelerations are averages and never exceed those of the input. The input is
// extended at rest on both sides, so the result starts and ends at the same positions, at rest,
// and is both windows longer. All joints use the same windows, so the path stays close to the
// input's; corners and blends are rounded a little more.
//
// The input is interpolated with cubic Hermite splines (positions and velocities); its
// accelerations are not used. Parameters: the same as move_group (robot_description,
// robot_description_semantic, robot_description_planning with the jerk limits), see
// move_group.launch.py.

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotarm_interface/srv/smooth_trajectory.hpp>

using robotarm_interface::srv::SmoothTrajectory;
using trajectory_msgs::msg::JointTrajectory;

namespace
{
constexpr double GRID_DT = 0.001;    // [s] integration grid
constexpr double OUTPUT_DT = 0.02;   // [s] spacing of the output points
constexpr double PREFILTER_WINDOW = 0.1;  // [s]
constexpr double MIN_WINDOW = 0.01;        // [s]

double seconds(const builtin_interfaces::msg::Duration& d)
{
  return d.sec + d.nanosec * 1e-9;
}

builtin_interfaces::msg::Duration duration(double t)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(t));
  d.nanosec = static_cast<uint32_t>(std::round((t - d.sec) * 1e9));
  if (d.nanosec >= 1000000000u)
  {
    d.sec += 1;
    d.nanosec -= 1000000000u;
  }
  return d;
}

// Moving average over w grid steps of a signal extended by `before` / `after` outside its range;
// the result is w - 1 steps longer, element i covers the input steps (i - w, i].
std::vector<double> movingAverage(const std::vector<double>& g, size_t w, double before, double after)
{
  const long n = static_cast<long>(g.size());
  const auto at = [&](long i) { return i < 0 ? before : (i >= n ? after : g[i]); };
  std::vector<double> out(g.size() + w - 1);
  double sum = w * before;
  for (long i = 0; i < static_cast<long>(out.size()); ++i)
  {
    sum += at(i) - at(i - static_cast<long>(w));
    out[i] = sum / w;
  }
  return out;
}

// Cubic Hermite interpolation of one joint (positions and velocities at the waypoints), held at the
// first / last position outside [t_0, t_end].
class HermiteJoint
{
public:
  HermiteJoint(const std::vector<double>& t, std::vector<double> q, std::vector<double> v)
    : t_(t), q_(std::move(q)), v_(std::move(v))
  {
  }

  // position, velocity, acceleration at time t
  void eval(double t, double& q, double& v, double& a) const
  {
    if (t <= t_.front() || t >= t_.back())
    {
      q = t <= t_.front() ? q_.front() : q_.back();
      v = a = 0.0;
      return;
    }
    const size_t k = std::upper_bound(t_.begin(), t_.end(), t) - t_.begin() - 1;
    const double h = t_[k + 1] - t_[k];
    const double s = (t - t_[k]) / h;
    const double p0 = q_[k], p1 = q_[k + 1], m0 = v_[k] * h, m1 = v_[k + 1] * h;
    // basis functions h00, h10, h01, h11 and their derivatives with respect to s
    const double s2 = s * s, s3 = s2 * s;
    q = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
    v = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1) / h;
    a = ((12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1) / (h * h);
  }

private:
  const std::vector<double>& t_;
  std::vector<double> q_, v_;
};
}  // namespace

class TrajectorySmoother
{
public:
  explicit TrajectorySmoother(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    // reads robot_description(_semantic) and the limits in robot_description_planning
    robot_model_loader::RobotModelLoader loader(node_, "robot_description", false);
    model_ = loader.getModel();
    if (!model_)
    {
      throw std::runtime_error("robot model not available");
    }
    service_ = node_->create_service<SmoothTrajectory>(
        "smooth_trajectory", [this](const std::shared_ptr<SmoothTrajectory::Request> req,
                                    std::shared_ptr<SmoothTrajectory::Response> res) {
          res->success = smooth(req->trajectory, res->trajectory, res->message);
          if (res->success)
          {
            RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "%s", res->message.c_str());
          }
        });
    RCLCPP_INFO(node_->get_logger(), "ready: %s", service_->get_service_name());
  }

private:
  bool smooth(const JointTrajectory& in, JointTrajectory& out, std::string& message) const
  {
    const size_t n_joints = in.joint_names.size();
    const size_t n_points = in.points.size();
    if (n_points < 2 || n_joints == 0)
    {
      message = "trajectory with less than 2 points";
      return false;
    }

    // jerk limits of the joints
    std::vector<double> max_jerk(n_joints);
    for (size_t j = 0; j < n_joints; ++j)
    {
      if (!model_->hasJointModel(in.joint_names[j]))
      {
        message = "unknown joint " + in.joint_names[j];
        return false;
      }
      const auto& bounds = model_->getJointModel(in.joint_names[j])->getVariableBounds()[0];
      if (!bounds.jerk_bounded_ || bounds.max_jerk_ <= 0.0)
      {
        message = "no jerk limit for " + in.joint_names[j] + " (joint_limits.yaml)";
        return false;
      }
      max_jerk[j] = bounds.max_jerk_;
    }

    // input as Hermite splines
    std::vector<double> t(n_points);
    std::vector<std::vector<double>> q(n_joints, std::vector<double>(n_points));
    std::vector<std::vector<double>> v(n_joints, std::vector<double>(n_points));
    for (size_t k = 0; k < n_points; ++k)
    {
      const auto& p = in.points[k];
      if (p.positions.size() != n_joints || p.velocities.size() != n_joints)
      {
        message = "point " + std::to_string(k) + " without positions or velocities for all joints";
        return false;
      }
      t[k] = seconds(p.time_from_start);
      if (k > 0 && t[k] <= t[k - 1])
      {
        message = "time_from_start not increasing at point " + std::to_string(k);
        return false;
      }
      for (size_t j = 0; j < n_joints; ++j)
      {
        q[j][k] = p.positions[j];
        v[j][k] = p.velocities[j];
      }
    }
    std::vector<HermiteJoint> splines;
    for (size_t j = 0; j < n_joints; ++j)
    {
      splines.emplace_back(t, q[j], v[j]);
    }

    // input on the grid [t_0, t_end]
    const double t0 = t.front();
    const size_t n_grid = static_cast<size_t>(std::ceil((t.back() - t0) / GRID_DT)) + 1;
    std::vector<std::vector<double>> gq(n_joints, std::vector<double>(n_grid));
    std::vector<std::vector<double>> gv(n_joints, std::vector<double>(n_grid));
    std::vector<std::vector<double>> ga(n_joints, std::vector<double>(n_grid));
    for (size_t j = 0; j < n_joints; ++j)
    {
      for (size_t i = 0; i < n_grid; ++i)
      {
        splines[j].eval(t0 + i * GRID_DT, gq[j][i], gv[j][i], ga[j][i]);
      }
    }

    // filter all joints with a window of `window` seconds (q held, v and a at rest outside);
    // returns the window rounded to the grid
    const auto filter = [&](double window) {
      const size_t w = std::max<size_t>(1, static_cast<size_t>(std::ceil(window / GRID_DT - 1e-9)));
      for (size_t j = 0; j < n_joints; ++j)
      {
        gq[j] = movingAverage(gq[j], w, gq[j].front(), gq[j].back());
        gv[j] = movingAverage(gv[j], w, 0.0, 0.0);
        ga[j] = movingAverage(ga[j], w, 0.0, 0.0);
      }
      return w * GRID_DT;
    };
    const double prefilter = filter(PREFILTER_WINDOW);
    double window = MIN_WINDOW;
    for (size_t j = 0; j < n_joints; ++j)
    {
      const auto [a_min, a_max] = std::minmax_element(ga[j].begin(), ga[j].end());
      window = std::max(window, (std::max(*a_max, 0.0) - std::min(*a_min, 0.0)) / max_jerk[j]);
    }
    window = filter(window);

    // output points every OUTPUT_DT and at the end
    const size_t n_out_grid = gq[0].size();
    const size_t step = static_cast<size_t>(std::round(OUTPUT_DT / GRID_DT));
    out = JointTrajectory();
    out.header = in.header;
    out.joint_names = in.joint_names;
    for (size_t i = 0; i < n_out_grid; ++i)
    {
      if (i % step != 0 && i != n_out_grid - 1)
      {
        continue;
      }
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.time_from_start = duration(t0 + i * GRID_DT);
      for (size_t j = 0; j < n_joints; ++j)
      {
        p.positions.push_back(gq[j][i]);
        p.velocities.push_back(gv[j][i]);
        p.accelerations.push_back(ga[j][i]);
      }
      out.points.push_back(std::move(p));
    }
    // exactly at rest at the goal
    auto& last = out.points.back();
    for (size_t j = 0; j < n_joints; ++j)
    {
      last.positions[j] = q[j].back();
      last.velocities[j] = last.accelerations[j] = 0.0;
    }

    std::ostringstream ss;
    ss.precision(3);
    ss << std::fixed << "smoothed " << n_points << " -> " << out.points.size() << " points, windows "
       << prefilter << " + " << window << " s, duration " << t.back() - t0 << " s -> " << seconds(last.time_from_start) - t0
       << " s";
    message = ss.str();
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  rclcpp::Service<SmoothTrajectory>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // like move_group: the MoveIt parameters (robot_description_planning.joint_limits...) are not
  // declared one by one
  auto node = std::make_shared<rclcpp::Node>(
      "trajectory_smoother", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  TrajectorySmoother smoother(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
