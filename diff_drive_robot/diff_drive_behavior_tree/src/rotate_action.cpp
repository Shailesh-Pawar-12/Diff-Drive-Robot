#include "diff_drive_behavior_tree/rotate_action.hpp"

#include "behaviortree_ros2/plugins.hpp"

bool RotateAction::setGoal(RosActionNode::Goal& goal) {
  RCLCPP_INFO(
      node_->get_logger(), ANSI_COLOR_GREEN
      "\33[1m Setting Goal for Rotate Action ... \33[0m" ANSI_COLOR_RESET);

  getInput("target_yaw", goal.target_yaw);
  getInput("control_angular_speed", goal.control_angular_speed);
  getInput("tolerance", goal.tolerance);
  getInput("velocity_profile", goal.velocity_profile);

  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  return true;
}

NodeStatus RotateAction::onResultReceived(
    const RosActionNode::WrappedResult& wr) {
  RCLCPP_INFO(
      node_->get_logger(), ANSI_COLOR_GREEN
      "\33[1m Rotate Action Successfully completed \33[0m" ANSI_COLOR_RESET);
  return NodeStatus::SUCCESS;
}

NodeStatus RotateAction::onFailure(ActionNodeErrorCode error) {
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return NodeStatus::FAILURE;
}

NodeStatus RotateAction::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(node_->get_logger(),
              ANSI_COLOR_PURPLE
              "\33[1m Remaining yaw: %f \33[0m" ANSI_COLOR_RESET,
              feedback->angular_distance_remaining);

  return NodeStatus::RUNNING;
}

void RotateAction::on_pause(Goal& goal,
                            const std::shared_ptr<const Feedback> feedback) {
  goal.target_yaw = feedback->angular_distance_remaining;
  RCLCPP_INFO(node_->get_logger(), "Paused");
  RCLCPP_INFO(node_->get_logger(),
              ANSI_COLOR_PURPLE
              "\33[1m Remaining yaw: %f \33[0m" ANSI_COLOR_RESET,
              goal.target_yaw);
}

// Plugin registration.
// The class RotateAction will self register with name  "RotateAction".
CreateRosNodePlugin(RotateAction, "RotateAction");
