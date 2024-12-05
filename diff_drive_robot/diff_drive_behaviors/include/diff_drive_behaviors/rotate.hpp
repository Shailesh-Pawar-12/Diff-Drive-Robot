#ifndef BEHAVIORS__PLUGINS__ROTATE_HPP_
#define BEHAVIORS__PLUGINS__ROTATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "bt_interfaces/action/rotate.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "timed_behavior.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace diff_drive_behaviors {
using RotateAction = bt_interfaces::action::Rotate;

/**
 * @class diff_drive_behaviors::Rotate
 * @brief An action server behavior for spinning in
 */
class Rotate : public nav2_behaviors::TimedBehavior<RotateAction> {
 public:
  /**
   * @brief A constructor for diff_drive_behaviors::Rotate
   */
  Rotate();
  ~Rotate();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return nav2_behaviors::Status of behavior
   */
  nav2_behaviors::Status onRun(
      const std::shared_ptr<const RotateAction::Goal> command) override;

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

  RotateAction::Feedback::SharedPtr feedback_;

  double command_yaw_;
  double command_angular_speed_;
  double tolerance_;
  double min_rotational_vel_;
  double max_rotational_vel_;
  double min_rotational_acc_;
  double max_rotational_acc_;
  double min_rotational_jerk_;
  double max_rotational_jerk_;
  double min_approach_angular_velocity_;
  uint8_t velocity_profile_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Duration total_elapsed_time_{0, 0};
  rclcpp::Time start_time_;
  rclcpp::Time deadline_time_;
  double allowable_positive_travel_angular_distance_;
  double allowable_negative_travel_angular_distance_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker marker_;
};

}  // namespace diff_drive_behaviors

#endif  // BEHAVIORS__PLUGINS__ROTATE_HPP_
