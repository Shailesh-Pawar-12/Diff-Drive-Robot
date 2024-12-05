#ifndef DIFF_DRIVE_BEHAVIORS__PLUGINS__LINEAR_HPP_
#define DIFF_DRIVE_BEHAVIORS__PLUGINS__LINEAR_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "bt_interfaces/action/linear.hpp"
#include "nav2_util/node_utils.hpp"
#include "timed_behavior.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace diff_drive_behaviors {
using LinearAction = bt_interfaces::action::Linear;

/**
 * @class diff_drive_behaviors::Linear
 * @brief An action server behavior for spinning in
 */
class Linear : public nav2_behaviors::TimedBehavior<LinearAction> {
 public:
  /**
   * @brief A constructor for diff_drive_behaviors::Linear
   */
  Linear();
  ~Linear();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return nav2_behaviors::Status of behavior
   */
  nav2_behaviors::Status onRun(
      const std::shared_ptr<const LinearAction::Goal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return nav2_behaviors::Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

 protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(const double& distance,
                       geometry_msgs::msg::Twist* cmd_vel,
                       geometry_msgs::msg::Pose2D& pose2d);

  LinearAction::Feedback::SharedPtr feedback_;
  // typename ActionT::Result::SharedPtr result_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_x_;
  double command_speed_;
  double tolerance_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Duration total_elapsed_time_{0, 0};
  rclcpp::Time deadline_time_;
  double simulate_ahead_time_;
  double allowable_forward_travel_distance_;
  double allowable_backward_travel_distance_;
  double max_jerk_;
  double min_jerk_;
  double max_acc_;
  double min_acc_;
  double max_translational_vel_;
  double min_translational_vel_;
  double min_approach_velocity_;
  uint8_t velocity_profile_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker marker_;
};

}  // namespace diff_drive_behaviors

#endif  // DIFF_DRIVE_BEHAVIORS__PLUGINS__LINEAR_HPP_
