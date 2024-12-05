#include "diff_drive_behaviors/rotate.hpp"

using namespace std::chrono_literals;

namespace diff_drive_behaviors {

Rotate::Rotate()
    : nav2_behaviors::TimedBehavior<RotateAction>(),
      feedback_(std::make_shared<RotateAction::Feedback>()), command_yaw_(0.0),
      min_rotational_vel_(0.0), max_rotational_vel_(0.0),
      max_rotational_acc_(0.0), min_approach_angular_velocity_(0.05),
      prev_yaw_(0.0), relative_yaw_(0.0), simulate_ahead_time_(0.0) {}

Rotate::~Rotate() = default;

void Rotate::onConfigure() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "simulate_ahead_time",
                                               rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(node, "max_rotational_vel",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(node, "min_rotational_vel",
                                               rclcpp::ParameterValue(-0.5));
  node->get_parameter("min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
      node, "max_rotational_acc", rclcpp::ParameterValue(0.665483585));
  node->get_parameter("max_rotational_acc", max_rotational_acc_);
  // 1.33096717/2=0.665483585

  nav2_util::declare_parameter_if_not_declared(
      node, "min_rotational_acc", rclcpp::ParameterValue(-0.665483585));
  node->get_parameter("min_rotational_acc", min_rotational_acc_);

  nav2_util::declare_parameter_if_not_declared(
      node, "min_approach_angular_velocity", rclcpp::ParameterValue(0.05));
  node->get_parameter("min_approach_angular_velocity",
                      min_approach_angular_velocity_);

  nav2_util::declare_parameter_if_not_declared(
      node, "allowable_positive_travel_angular_distance",
      rclcpp::ParameterValue(10.0));
  node->get_parameter("allowable_positive_travel_angular_distance",
                      allowable_positive_travel_angular_distance_);

  nav2_util::declare_parameter_if_not_declared(
      node, "allowable_negative_travel_angular_distance",
      rclcpp::ParameterValue(5.0));
  node->get_parameter("allowable_negative_travel_angular_distance",
                      allowable_negative_travel_angular_distance_);

  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
      "diff_drive_behaviors_visualization_marker", 1);
}

nav2_behaviors::Status Rotate::onRun(
    const std::shared_ptr<const RotateAction::Goal> command) {
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);

  relative_yaw_ = 0.0;

  command_yaw_ = command->target_yaw;
  RCLCPP_INFO(logger_, "Turning %0.2f for spin behavior.", command_yaw_);

  command_angular_speed_ = std::fabs(command->control_angular_speed);
  tolerance_ = std::fabs(command->tolerance);
  velocity_profile_ = command->velocity_profile;
  command_time_allowance_ = command->time_allowance;
  deadline_time_ = steady_clock_.now() + command_time_allowance_;
  min_approach_angular_velocity_ = std::fabs(min_approach_angular_velocity_);

  tf2::Quaternion q_start, q_rot, q_end;
  tf2::fromMsg(current_pose.pose.orientation, q_start);
  q_rot.setRPY(0.0, 0.0, command_yaw_);
  q_end = q_rot * q_start;
  q_end.normalize();

  if (command_time_allowance_.seconds() <= 0.0) {
    RCLCPP_INFO(this->logger_,
                "Invalid(negative or zero) allowed time goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_yaw_ == 0.0) {
    RCLCPP_INFO(this->logger_, "Invalid(zero) command_yaw goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (tolerance_ >= std::fabs(command_yaw_)) {
    RCLCPP_INFO(this->logger_,
                "Invalid(within tolerance) command_yaw goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_angular_speed_ == 0.0) {
    RCLCPP_INFO(this->logger_,
                "Invalid(zero) command_angular_speed goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_angular_speed_ < min_approach_angular_velocity_) {
    RCLCPP_INFO(this->logger_,
                "command_angular_speed is lower than command_angular_speed");
    return nav2_behaviors::Status::FAILED;
  } else if (command_yaw_ > 0.0) {
    if (command_yaw_ > std::fabs(allowable_positive_travel_angular_distance_)) {
      RCLCPP_INFO(this->logger_,
                  "Target yaw is more than allowable forward travel distance");
      return nav2_behaviors::Status::FAILED;
    } else if (command_angular_speed_ > std::fabs(max_rotational_vel_)) {
      RCLCPP_INFO(
          this->logger_,
          "Command velocity is more than allowable max_rotational_vel_ param");
      return nav2_behaviors::Status::FAILED;
    }
  } else if (command_yaw_ < 0.0) {
    if (std::fabs(command_yaw_) >
        std::fabs(allowable_negative_travel_angular_distance_)) {
      RCLCPP_INFO(this->logger_,
                  "Target yaw is more than allowable backward travel distance");
      return nav2_behaviors::Status::FAILED;
    } else if (command_angular_speed_ > std::fabs(min_rotational_vel_)) {
      RCLCPP_INFO(this->logger_,
                  "Command velocity is less than allowable min velocity");
      return nav2_behaviors::Status::FAILED;
    }
  }

  // Setup the marker
  marker_pub_->on_activate();

  marker_.type = visualization_msgs::msg::Marker::ARROW;
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.scale.x = 0.35;
  marker_.scale.y = 0.1;
  marker_.scale.z = 1.0;
  marker_.color.r = 0.0;
  marker_.color.g = 0.0;
  marker_.color.b = 1.0;
  marker_.color.a = 0.5;
  marker_.lifetime = rclcpp::Duration(25000ms);
  marker_.frame_locked = true;

  marker_.header.frame_id = this->global_frame_;
  marker_.ns = "rotate";
  marker_.id = 16000;
  marker_.pose = current_pose.pose;
  marker_.pose.orientation = tf2::toMsg(q_end);
  marker_.lifetime = command_time_allowance_;

  marker_pub_->publish(marker_);

  marker_pub_->on_deactivate();

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status Rotate::onCycleUpdate() {
  rclcpp::Duration time_remaining = deadline_time_ - this->steady_clock_.now();
  if (time_remaining.seconds() < 0.0) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_,
                "Exceeded time allowance before reaching the Rotate goal - "
                "Exiting Rotate");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  double delta_yaw = current_yaw - prev_yaw_;
  if (abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
  }

  relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;

  double remaining_yaw = command_yaw_ - relative_yaw_;

  feedback_->angular_distance_traveled = relative_yaw_;
  feedback_->angular_distance_remaining = remaining_yaw;
  action_server_->publish_feedback(feedback_);

  if (command_yaw_ > 0.0) {
    if (remaining_yaw < tolerance_) {
      this->stopRobot();
      return nav2_behaviors::Status::SUCCEEDED;
    }
  } else if (command_yaw_ < 0.0) {
    if (remaining_yaw > -tolerance_) {
      this->stopRobot();
      return nav2_behaviors::Status::SUCCEEDED;
    }
  }

  double vel;
  switch (velocity_profile_) {
    case 0:
      vel = command_angular_speed_;
      break;
    case 1:
      vel = std::max(std::min(std::fabs(remaining_yaw / command_yaw_) *
                                  command_angular_speed_,
                              command_angular_speed_),
                     min_approach_angular_velocity_);
      break;
    case 2:
      vel = std::max(std::min(sqrt(std::fabs(remaining_yaw / command_yaw_)) *
                                  command_angular_speed_,
                              command_angular_speed_),
                     min_approach_angular_velocity_);
      break;
    default:
      vel = command_angular_speed_;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->angular.z = copysign(std::fabs(vel), command_yaw_);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  // if (!isCollisionFree(relative_yaw_, cmd_vel.get(), pose2d)) {
  //   this->stopRobot();
  //   RCLCPP_WARN(logger_, "Collision Ahead - Exiting Rotate");
  //   return nav2_behaviors::Status::FAILED;
  // }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

bool Rotate::isCollisionFree(const double& relative_yaw,
                             geometry_msgs::msg::Twist* cmd_vel,
                             geometry_msgs::msg::Pose2D& pose2d) {
  // Simulate ahead by simulate_ahead_time_ in cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  const int max_cycle_count =
      static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel->angular.z * (cycle_count / cycle_frequency_);
    pose2d.theta = init_pose.theta + sim_position_change;
    cycle_count++;

    if (abs(relative_yaw) - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace diff_drive_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_drive_behaviors::Rotate, nav2_core::Behavior)
