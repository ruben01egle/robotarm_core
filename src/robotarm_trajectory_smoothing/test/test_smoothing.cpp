// Unit tests of the jerk-limited trajectory smoothing (smoothing.hpp).
//
// The inputs are rest-to-rest motions with known profiles, sampled every 0.1 s like Pilz. Jerk is
// checked the way joint_trajectory_controller executes the result: quintic splines through
// position, velocity and acceleration of the output points.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "robotarm_trajectory_smoothing/smoothing.hpp"

using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;
namespace sm = robotarm_trajectory_smoothing;

namespace
{
constexpr double PILZ_DT = 0.1;  // [s] sampling of the inputs

struct State
{
  double q, v, a;
};
using Profile = std::function<State(double)>;

// Rest-to-rest motion over `distance` with trapezoidal velocity (triangular if too short).
struct Trapezoid
{
  double distance, v_max, a_max, start = 0.0;

  double duration() const
  {
    const double d = std::abs(distance);
    const double t_acc = v_max / a_max;
    return d >= v_max * t_acc ? d / v_max + t_acc : 2 * std::sqrt(d / a_max);
  }

  State operator()(double t) const
  {
    const double sign = distance < 0 ? -1.0 : 1.0;
    const double T = duration();
    const double t_acc = std::min(v_max / a_max, T / 2);
    const double v_top = a_max * t_acc;
    State s{ 0, 0, 0 };
    if (t <= 0)
    {
      s = { 0, 0, 0 };
    }
    else if (t >= T)
    {
      s = { std::abs(distance), 0, 0 };
    }
    else if (t < t_acc)
    {
      s = { a_max * t * t / 2, a_max * t, a_max };
    }
    else if (t <= T - t_acc)
    {
      s = { v_top * t_acc / 2 + v_top * (t - t_acc), v_top, 0 };
    }
    else
    {
      const double r = T - t;
      s = { std::abs(distance) - a_max * r * r / 2, a_max * r, -a_max };
    }
    return { start + sign * s.q, sign * s.v, sign * s.a };
  }
};

// Trajectory of one or more joints from profiles, sampled every PILZ_DT and at the end.
JointTrajectory makeTrajectory(const std::vector<Profile>& joints, double duration, double t0 = 0.0)
{
  JointTrajectory traj;
  for (size_t j = 0; j < joints.size(); ++j)
  {
    traj.joint_names.push_back("j" + std::to_string(j + 1));
  }
  std::vector<double> times;
  for (double t = 0.0; t < duration - 1e-9; t += PILZ_DT)
  {
    times.push_back(t);
  }
  times.push_back(duration);
  for (const double t : times)
  {
    JointTrajectoryPoint p;
    const double ts = t0 + t;
    p.time_from_start.sec = static_cast<int32_t>(std::floor(ts));
    p.time_from_start.nanosec = static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
    for (const auto& profile : joints)
    {
      const State s = profile(t);
      p.positions.push_back(s.q);
      p.velocities.push_back(s.v);
      p.accelerations.push_back(s.a);
    }
    traj.points.push_back(p);
  }
  return traj;
}

// Largest |jerk| of the quintic splines through the points (as joint_trajectory_controller).
double maxQuinticJerk(const JointTrajectory& traj, size_t joint)
{
  double max_jerk = 0.0;
  for (size_t k = 0; k + 1 < traj.points.size(); ++k)
  {
    const auto& a = traj.points[k];
    const auto& b = traj.points[k + 1];
    const double T = sm::seconds(b.time_from_start) - sm::seconds(a.time_from_start);
    const double p0 = a.positions[joint], p1 = b.positions[joint];
    const double v0 = a.velocities[joint], v1 = b.velocities[joint];
    const double a0 = a.accelerations[joint], a1 = b.accelerations[joint];
    const double c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T * T) / (2 * T * T * T);
    const double c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T * T) / (2 * std::pow(T, 4));
    const double c5 = (12 * (p1 - p0) - 6 * (v1 + v0) * T + (a1 - a0) * T * T) / (2 * std::pow(T, 5));
    for (int i = 0; i <= 20; ++i)
    {
      const double s = T * i / 20;
      max_jerk = std::max(max_jerk, std::abs(6 * c3 + 24 * c4 * s + 60 * c5 * s * s));
    }
  }
  return max_jerk;
}

double maxAbs(const JointTrajectory& traj, size_t joint, bool velocity)
{
  double m = 0.0;
  for (const auto& p : traj.points)
  {
    m = std::max(m, std::abs(velocity ? p.velocities[joint] : p.accelerations[joint]));
  }
  return m;
}

bool atRest(const JointTrajectoryPoint& p)
{
  for (size_t j = 0; j < p.velocities.size(); ++j)
  {
    if (p.velocities[j] != 0.0 || p.accelerations[j] != 0.0)
    {
      return false;
    }
  }
  return true;
}

JointTrajectory smoothOk(const JointTrajectory& in, const std::vector<double>& max_jerk,
                         sm::SmoothingInfo* info = nullptr)
{
  JointTrajectory out;
  std::string message;
  EXPECT_TRUE(sm::smoothTrajectory(in, max_jerk, out, message, info)) << message;
  return out;
}

// two moves with a stop in between: 0 -> 1 rad, then 1 -> 0.4 rad
JointTrajectory twoMovesWithStop(double& stop_position)
{
  const Trapezoid first{ 1.0, 0.8, 4.0 };
  const Trapezoid second{ -0.6, 0.8, 4.0, 1.0 };
  stop_position = 1.0;
  JointTrajectory a = makeTrajectory({ first }, first.duration());
  const JointTrajectory b = makeTrajectory({ second }, second.duration(), first.duration());
  a.points.insert(a.points.end(), b.points.begin() + 1, b.points.end());  // stop point shared
  return a;
}
}  // namespace

TEST(Smoothing, StartsAndEndsExactlyAtRest)
{
  const Trapezoid move{ 1.2, 0.8, 4.0, 0.3 };
  const auto out = smoothOk(makeTrajectory({ move }, move.duration()), { 15.0 });
  ASSERT_GE(out.points.size(), 2u);
  EXPECT_EQ(out.points.front().positions[0], 0.3);
  EXPECT_EQ(out.points.back().positions[0], 1.5);
  EXPECT_TRUE(atRest(out.points.front()));
  EXPECT_TRUE(atRest(out.points.back()));
}

TEST(Smoothing, KeepsJerkLimitBetweenPoints)
{
  // triangular profile: the acceleration jumps from +a to -a, the hardest case for the window
  for (const double limit : { 5.0, 15.0, 40.0 })
  {
    const Trapezoid move{ 0.3, 2.0, 4.0 };
    const JointTrajectory in = makeTrajectory({ move }, move.duration());
    EXPECT_GT(maxQuinticJerk(in, 0), 10 * limit) << "the input should violate the limit";
    const auto out = smoothOk(in, { limit });
    EXPECT_LE(maxQuinticJerk(out, 0), limit * 1.02) << "limit " << limit;
  }
}

TEST(Smoothing, NeverFasterOrHarderThanInput)
{
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const JointTrajectory in = makeTrajectory({ move }, move.duration());
  const auto out = smoothOk(in, { 15.0 });
  EXPECT_LE(maxAbs(out, 0, true), 0.8 * 1.01);
  EXPECT_LE(maxAbs(out, 0, false), 4.0 * 1.01);
}

TEST(Smoothing, ConsistentDerivatives)
{
  // position steps match velocity and acceleration (quintic-exact integration of the samples)
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const auto out = smoothOk(makeTrajectory({ move }, move.duration()), { 15.0 });
  for (size_t k = 0; k + 1 < out.points.size(); ++k)
  {
    const auto& a = out.points[k];
    const auto& b = out.points[k + 1];
    const double T = sm::seconds(b.time_from_start) - sm::seconds(a.time_from_start);
    const double predicted = a.positions[0] + (a.velocities[0] + b.velocities[0]) / 2 * T -
                             (b.accelerations[0] - a.accelerations[0]) / 12 * T * T;
    EXPECT_NEAR(b.positions[0], predicted, 1e-6) << "point " << k;
  }
}

TEST(Smoothing, OutputTiming)
{
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const JointTrajectory in = makeTrajectory({ move }, move.duration(), 0.5);  // starts at 0.5 s
  sm::SmoothingInfo info;
  const auto out = smoothOk(in, { 15.0 }, &info);
  EXPECT_NEAR(sm::seconds(out.points.front().time_from_start), 0.5, 1e-9);
  for (size_t k = 0; k + 1 < out.points.size(); ++k)
  {
    const double dt = sm::seconds(out.points[k + 1].time_from_start) - sm::seconds(out.points[k].time_from_start);
    EXPECT_GT(dt, 0.0);
    EXPECT_LE(dt, sm::OUTPUT_DT + 1e-9);
  }
  // one piece: longer by the two windows (to the grid)
  EXPECT_EQ(info.pieces, 1u);
  EXPECT_NEAR(info.duration_in, move.duration(), 1e-9);
  EXPECT_NEAR(info.duration_out, info.duration_in + sm::PREFILTER_WINDOW + info.max_window, 2 * sm::GRID_DT);
  EXPECT_NEAR(sm::seconds(out.points.back().time_from_start), 0.5 + info.duration_out, 1e-6);
}

TEST(Smoothing, LowerJerkLimitGivesLongerWindow)
{
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const JointTrajectory in = makeTrajectory({ move }, move.duration());
  sm::SmoothingInfo soft, hard;
  smoothOk(in, { 5.0 }, &soft);
  smoothOk(in, { 50.0 }, &hard);
  EXPECT_GT(soft.max_window, hard.max_window);
  EXPECT_GT(soft.duration_out, hard.duration_out);
  // window = acceleration range / jerk limit: the trapezoid goes from +4 to -4 rad/s^2. The
  // Hermite interpolation of the 0.1 s samples overshoots the acceleration corners a little, so
  // the window may be slightly longer, never shorter.
  EXPECT_GE(soft.max_window, 8.0 / 5.0 - sm::GRID_DT);
  EXPECT_LE(soft.max_window, 8.0 / 5.0 * 1.03);
}

TEST(Smoothing, StopsAreExact)
{
  double stop = 0.0;
  const JointTrajectory in = twoMovesWithStop(stop);
  sm::SmoothingInfo info;
  const auto out = smoothOk(in, { 15.0 }, &info);
  EXPECT_EQ(info.pieces, 2u);
  int exact_stops = 0;
  for (size_t k = 1; k + 1 < out.points.size(); ++k)
  {
    if (atRest(out.points[k]))
    {
      EXPECT_EQ(out.points[k].positions[0], stop);
      ++exact_stops;
    }
  }
  EXPECT_EQ(exact_stops, 1);
  // the stop is not overshot: the first move never goes beyond it
  for (const auto& p : out.points)
  {
    EXPECT_LE(p.positions[0], stop + 1e-12);
  }
  EXPECT_LE(maxQuinticJerk(out, 0), 15.0 * 1.02);
}

TEST(Smoothing, BlendIsNotSplit)
{
  // two overlapping moves: the velocity dips between them but never reaches zero
  const Trapezoid first{ 1.0, 0.8, 4.0 };
  const Trapezoid second{ 0.5, 0.8, 4.0 };
  const double overlap = 0.6;
  const double shift = first.duration() - overlap;
  const Profile blended = [&](double t) {
    const State a = first(t), b = second(t - shift);
    return State{ a.q + b.q, a.v + b.v, a.a + b.a };
  };
  sm::SmoothingInfo info;
  const auto out = smoothOk(makeTrajectory({ blended }, shift + second.duration()), { 15.0 }, &info);
  EXPECT_EQ(info.pieces, 1u);
  EXPECT_NEAR(out.points.back().positions[0], 1.5, 1e-12);
}

TEST(Smoothing, AllJointsShareTheWindowSoThePathIsKept)
{
  // joint 2 follows joint 1 linearly (a straight line in joint space); joint 2 has the tighter
  // jerk limit and sets the window. The result must stay on the same line.
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const Profile follower = [&](double t) {
    const State s = move(t);
    return State{ 2 * s.q + 0.5, 2 * s.v, 2 * s.a };
  };
  sm::SmoothingInfo info;
  const auto out = smoothOk(makeTrajectory({ move, follower }, move.duration()), { 50.0, 10.0 }, &info);
  for (const auto& p : out.points)
  {
    EXPECT_NEAR(p.positions[1], 2 * p.positions[0] + 0.5, 1e-9);
  }
  // joint 2 sets the window: range 16 rad/s^2, limit 10 (slightly longer, see above)
  EXPECT_GE(info.max_window, 16.0 / 10.0 - sm::GRID_DT);
  EXPECT_LE(info.max_window, 16.0 / 10.0 * 1.03);
}

TEST(Smoothing, InputAccelerationsAreIgnored)
{
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const JointTrajectory clean = makeTrajectory({ move }, move.duration());
  JointTrajectory noisy = clean;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> noise(-50.0, 50.0);
  for (auto& p : noisy.points)
  {
    p.accelerations[0] = noise(rng);
  }
  const auto a = smoothOk(clean, { 15.0 });
  const auto b = smoothOk(noisy, { 15.0 });
  ASSERT_EQ(a.points.size(), b.points.size());
  for (size_t k = 0; k < a.points.size(); ++k)
  {
    EXPECT_EQ(a.points[k].positions[0], b.points[k].positions[0]);
    EXPECT_EQ(a.points[k].accelerations[0], b.points[k].accelerations[0]);
  }
}

TEST(Smoothing, RejectsInvalidInput)
{
  const Trapezoid move{ 1.0, 0.8, 4.0 };
  const JointTrajectory good = makeTrajectory({ move }, move.duration());
  JointTrajectory out;
  std::string message;

  JointTrajectory one_point = good;
  one_point.points.resize(1);
  EXPECT_FALSE(sm::smoothTrajectory(one_point, { 15.0 }, out, message));

  JointTrajectory bad_time = good;
  bad_time.points[3].time_from_start = bad_time.points[2].time_from_start;
  EXPECT_FALSE(sm::smoothTrajectory(bad_time, { 15.0 }, out, message));
  EXPECT_NE(message.find("not increasing"), std::string::npos);

  JointTrajectory no_velocities = good;
  no_velocities.points[4].velocities.clear();
  EXPECT_FALSE(sm::smoothTrajectory(no_velocities, { 15.0 }, out, message));

  JointTrajectory moving_at_end = good;
  moving_at_end.points.back().velocities[0] = 0.5;
  EXPECT_FALSE(sm::smoothTrajectory(moving_at_end, { 15.0 }, out, message));
  EXPECT_NE(message.find("at rest"), std::string::npos);

  EXPECT_FALSE(sm::smoothTrajectory(good, {}, out, message));
  EXPECT_FALSE(sm::smoothTrajectory(good, { 0.0 }, out, message));
  EXPECT_FALSE(sm::smoothTrajectory(good, { std::nan("") }, out, message));
}
