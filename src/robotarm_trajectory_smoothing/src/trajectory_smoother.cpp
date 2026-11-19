// Node trajectory_smoother: jerk-limited smoothing of whole joint trajectories, service
// /smooth_trajectory (robotarm_interface/srv/SmoothTrajectory). The algorithm is in smoothing.cpp
// (see smoothing.hpp); this node only reads the jerk limits from the robot model.
//
// Parameters: the same as move_group (robot_description, robot_description_semantic,
// robot_description_planning with the jerk limits of joint_limits.yaml), see move_group.launch.py.

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotarm_interface/srv/smooth_trajectory.hpp>

#include "robotarm_trajectory_smoothing/smoothing.hpp"

using robotarm_interface::srv::SmoothTrajectory;
using trajectory_msgs::msg::JointTrajectory;
namespace smoothing = robotarm_trajectory_smoothing;

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
    // jerk limits of the joints (joint_limits.yaml)
    std::vector<double> max_jerk;
    for (const auto& name : in.joint_names)
    {
      if (!model_->hasJointModel(name))
      {
        message = "unknown joint " + name;
        return false;
      }
      const auto& bounds = model_->getJointModel(name)->getVariableBounds()[0];
      if (!bounds.jerk_bounded_ || bounds.max_jerk_ <= 0.0)
      {
        message = "no jerk limit for " + name + " (joint_limits.yaml)";
        return false;
      }
      max_jerk.push_back(bounds.max_jerk_);
    }

    smoothing::SmoothingInfo info;
    if (!smoothing::smoothTrajectory(in, max_jerk, out, message, &info))
    {
      return false;
    }
    std::ostringstream ss;
    ss.precision(3);
    ss << std::fixed << "smoothed " << in.points.size() << " -> " << out.points.size() << " points, "
       << info.pieces << " piece(s) between stops, windows " << smoothing::PREFILTER_WINDOW << " + "
       << info.min_window;
    if (info.max_window > info.min_window)
    {
      ss << ".." << info.max_window;
    }
    ss << " s, duration " << info.duration_in << " s -> " << info.duration_out << " s";
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
