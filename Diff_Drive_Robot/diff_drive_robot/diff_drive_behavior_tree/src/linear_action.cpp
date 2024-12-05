#include "diff_drive_behavior_tree/linear_action.hpp"

#include "behaviortree_ros2/plugins.hpp"

bool LinearAction::setGoal(RosActionNode::Goal& goal) {
  RCLCPP_INFO(
      node_->get_logger(), ANSI_COLOR_GREEN
      "\33[1m Setting Goal for Linear Action ... \33[0m" ANSI_COLOR_RESET);

  if (!getInput("target_x", goal.target_x)) {
    return false;
  }
  getInput("control_speed", goal.control_speed);
  getInput("tolerance", goal.tolerance);
  getInput("velocity_profile", goal.velocity_profile);
  double time_allowance;
  if (getInput("time_allowance", time_allowance)) {
    goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  }

  return true;
}

NodeStatus LinearAction::onResultReceived(
    const RosActionNode::WrappedResult& wr) {
  RCLCPP_INFO(
      node_->get_logger(), ANSI_COLOR_GREEN
      "\33[1m Linear Action Successfully completed \33[0m" ANSI_COLOR_RESET);
  return NodeStatus::SUCCESS;
}

NodeStatus LinearAction::onFailure(ActionNodeErrorCode error) {
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}

// void LinearAction::onHalt()
// {
//   RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
// }

NodeStatus LinearAction::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(node_->get_logger(),
              ANSI_COLOR_PURPLE
              "\33[1m Distance remaning : %f \33[0m" ANSI_COLOR_RESET,
              feedback->distance_remaining);

  return NodeStatus::RUNNING;
}

void LinearAction::on_pause(Goal& goal,
                            const std::shared_ptr<const Feedback> feedback) {
  goal.target_x = feedback->distance_remaining;
  RCLCPP_WARN(logger(), "Paused");
  RCLCPP_INFO(logger(),
              ANSI_COLOR_PURPLE
              "\33[1m Remaining x: %f \33[0m" ANSI_COLOR_RESET,
              goal.target_x);
}

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(LinearAction, "LinearAction");
