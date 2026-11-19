// Jerk-limited smoothing of whole joint trajectories, see smoothing.hpp.

#include "robotarm_trajectory_smoothing/smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace robotarm_trajectory_smoothing
{
using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;

double seconds(const builtin_interfaces::msg::Duration& d)
{
  return d.sec + d.nanosec * 1e-9;
}

namespace
{
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

// One joint on the grid: position, velocity, acceleration.
struct Signal
{
  std::vector<double> q, v, a;
};

// Moving average over w grid steps (T = w * GRID_DT) of a signal at rest outside its range (q held,
// v = 0). The derivatives come from the continuous formulas, not from averaging v and a as well:
//   q_s(t) = 1/T * integral of q over [t - T, t]   (cumulative trapezoid)
//   v_s(t) = (q(t) - q(t - T)) / T,  a_s(t) = (v(t) - v(t - T)) / T
// so q_s, v_s and a_s are exactly consistent (averaging a sampled acceleration with jumps is not,
// and the quintic splines of joint_trajectory_controller would turn the mismatch into jerk peaks).
// Uses only q and v of the input; the result is w steps longer.
Signal boxFilter(const Signal& in, size_t w)
{
  const size_t n = in.q.size();
  const double T = w * GRID_DT;
  // extended input: w steps at rest before and after
  std::vector<double> qe(n + 2 * w), ve(n + 2 * w, 0.0), Q(n + 2 * w, 0.0);
  for (size_t i = 0; i < qe.size(); ++i)
  {
    qe[i] = i < w ? in.q.front() : (i >= n + w ? in.q.back() : in.q[i - w]);
    if (i >= w && i < n + w)
    {
      ve[i] = in.v[i - w];
    }
    if (i > 0)
    {
      Q[i] = Q[i - 1] + (qe[i] + qe[i - 1]) / 2 * GRID_DT;
    }
  }
  Signal out;
  for (size_t i = 0; i < n + w; ++i)  // output i covers the extended input [i, i + w]
  {
    out.q.push_back((Q[i + w] - Q[i]) / T);
    out.v.push_back((qe[i + w] - qe[i]) / T);
    out.a.push_back((ve[i + w] - ve[i]) / T);
  }
  return out;
}

// Cubic Hermite interpolation of one joint (positions and velocities at the waypoints), held at the
// first / last position outside [t_0, t_end].
class HermiteJoint
{
public:
  HermiteJoint(const std::vector<double>& t, const std::vector<double>& q, const std::vector<double>& v)
    : t_(t), q_(q), v_(v)
  {
  }

  // position and velocity at time t
  void eval(double t, double& q, double& v) const
  {
    if (t <= t_.front() || t >= t_.back())
    {
      q = t <= t_.front() ? q_.front() : q_.back();
      v = 0.0;
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
  }

private:
  const std::vector<double>& t_;
  const std::vector<double>& q_;
  const std::vector<double>& v_;
};

// Filter one piece that starts and ends at rest (times pt, positions pq and velocities pv per
// joint) and append its points to `points`, starting at time t_start. Returns the jerk window.
double smoothPiece(const std::vector<double>& pt, const std::vector<std::vector<double>>& pq,
                   const std::vector<std::vector<double>>& pv, const std::vector<double>& max_jerk,
                   double t_start, std::vector<JointTrajectoryPoint>& points)
{
  const size_t n_joints = pq.size();
  // input on the grid [t_0, t_end]
  const size_t n_grid = static_cast<size_t>(std::ceil((pt.back() - pt.front()) / GRID_DT)) + 1;
  std::vector<Signal> sig(n_joints);
  for (size_t j = 0; j < n_joints; ++j)
  {
    const HermiteJoint spline(pt, pq[j], pv[j]);
    sig[j].q.resize(n_grid);
    sig[j].v.resize(n_grid);
    for (size_t i = 0; i < n_grid; ++i)
    {
      spline.eval(pt.front() + i * GRID_DT, sig[j].q[i], sig[j].v[i]);
    }
  }

  // filter all joints with a window of `window` seconds; returns the window rounded to the grid
  const auto filter = [&](double window) {
    const size_t w = std::max<size_t>(1, static_cast<size_t>(std::ceil(window / GRID_DT - 1e-9)));
    for (auto& s : sig)
    {
      s = boxFilter(s, w);
    }
    return w * GRID_DT;
  };
  filter(PREFILTER_WINDOW);
  double window = MIN_WINDOW;
  for (size_t j = 0; j < n_joints; ++j)
  {
    const auto [a_min, a_max] = std::minmax_element(sig[j].a.begin(), sig[j].a.end());
    window = std::max(window, (std::max(*a_max, 0.0) - std::min(*a_min, 0.0)) / max_jerk[j]);
  }
  window = filter(window);

  // output points every OUTPUT_DT and at the end
  const size_t n_out_grid = sig[0].q.size();
  const size_t step = static_cast<size_t>(std::round(OUTPUT_DT / GRID_DT));
  for (size_t i = 0; i < n_out_grid; ++i)
  {
    if (i % step != 0 && i != n_out_grid - 1)
    {
      continue;
    }
    JointTrajectoryPoint p;
    p.time_from_start = duration(t_start + i * GRID_DT);
    for (size_t j = 0; j < n_joints; ++j)
    {
      p.positions.push_back(sig[j].q[i]);
      p.velocities.push_back(sig[j].v[i]);
      p.accelerations.push_back(sig[j].a[i]);
    }
    points.push_back(std::move(p));
  }
  // exactly at rest on the first and last point of the piece (start, stop or goal)
  for (size_t j = 0; j < n_joints; ++j)
  {
    points.front().positions[j] = pq[j].front();
    points.back().positions[j] = pq[j].back();
    points.front().velocities[j] = points.front().accelerations[j] = 0.0;
    points.back().velocities[j] = points.back().accelerations[j] = 0.0;
  }
  return window;
}
}  // namespace

bool smoothTrajectory(const JointTrajectory& in, const std::vector<double>& max_jerk, JointTrajectory& out,
                      std::string& message, SmoothingInfo* info)
{
  const size_t n_joints = in.joint_names.size();
  const size_t n_points = in.points.size();
  if (n_points < 2 || n_joints == 0)
  {
    message = "trajectory with less than 2 points";
    return false;
  }
  if (max_jerk.size() != n_joints)
  {
    message = "need one jerk limit per joint";
    return false;
  }
  for (size_t j = 0; j < n_joints; ++j)
  {
    if (!(max_jerk[j] > 0.0) || !std::isfinite(max_jerk[j]))
    {
      message = "no valid jerk limit for " + in.joint_names[j];
      return false;
    }
  }

  // input
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
  for (size_t j = 0; j < n_joints; ++j)
  {
    if (std::abs(v[j].front()) > END_VELOCITY || std::abs(v[j].back()) > END_VELOCITY)
    {
      message = "trajectory does not start and end at rest (" + in.joint_names[j] + ")";
      return false;
    }
  }

  // split at the stops: inner points where all joints are at rest (a step without blending)
  std::vector<size_t> bounds{ 0 };
  for (size_t k = 1; k + 1 < n_points; ++k)
  {
    bool at_rest = true;
    for (size_t j = 0; j < n_joints && at_rest; ++j)
    {
      at_rest = std::abs(v[j][k]) <= STOP_VELOCITY;
    }
    if (at_rest)
    {
      bounds.push_back(k);
    }
  }
  bounds.push_back(n_points - 1);

  // filter every piece on its own; each ends exactly at rest on its last point
  out = JointTrajectory();
  out.header = in.header;
  out.joint_names = in.joint_names;
  double t_out = t.front();
  std::vector<double> windows;
  for (size_t b = 0; b + 1 < bounds.size(); ++b)
  {
    const size_t first = bounds[b], last = bounds[b + 1];
    std::vector<double> pt(t.begin() + first, t.begin() + last + 1);
    std::vector<std::vector<double>> pq(n_joints), pv(n_joints);
    for (size_t j = 0; j < n_joints; ++j)
    {
      pq[j].assign(q[j].begin() + first, q[j].begin() + last + 1);
      pv[j].assign(v[j].begin() + first, v[j].begin() + last + 1);
    }
    std::vector<JointTrajectoryPoint> piece;
    windows.push_back(smoothPiece(pt, pq, pv, max_jerk, t_out, piece));
    // the first point of a piece is the last of the previous one (same position, at rest)
    out.points.insert(out.points.end(), piece.begin() + (out.points.empty() ? 0 : 1), piece.end());
    t_out = seconds(out.points.back().time_from_start);
  }

  if (info)
  {
    const auto [w_min, w_max] = std::minmax_element(windows.begin(), windows.end());
    info->pieces = windows.size();
    info->min_window = *w_min;
    info->max_window = *w_max;
    info->duration_in = t.back() - t.front();
    info->duration_out = t_out - t.front();
  }
  return true;
}

}  // namespace robotarm_trajectory_smoothing
