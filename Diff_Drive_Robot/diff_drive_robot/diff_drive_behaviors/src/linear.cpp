#include "diff_drive_behaviors/linear.hpp"

using namespace std::chrono_literals;

namespace diff_drive_behaviors {

Linear::Linear()
    : nav2_behaviors::TimedBehavior<LinearAction>(),
      feedback_(std::make_shared<LinearAction::Feedback>()),
      command_x_(0.0),
      command_speed_(0.15),
      simulate_ahead_time_(0.0),
      allowable_forward_travel_distance_(10.0),
      allowable_backward_travel_distance_(5.0),
      min_approach_velocity_(0.05),
      velocity_profile_(2) {}

Linear::~Linear() = default;

void Linear::onConfigure() {
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "simulate_ahead_time",
                                               rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(
      node, "allowable_forward_travel_distance", rclcpp::ParameterValue(10.0));
  node->get_parameter("allowable_forward_travel_distance",
                      allowable_forward_travel_distance_);

  nav2_util::declare_parameter_if_not_declared(
      node, "allowable_backward_travel_distance", rclcpp::ParameterValue(5.0));
  node->get_parameter("allowable_backward_travel_distance",
                      allowable_backward_travel_distance_);

  nav2_util::declare_parameter_if_not_declared(node, "max_jerk",
                                               rclcpp::ParameterValue(5.0));
  node->get_parameter("max_jerk", max_jerk_);

  nav2_util::declare_parameter_if_not_declared(node, "min_jerk",
                                               rclcpp::ParameterValue(-5.0));
  node->get_parameter("min_jerk", min_jerk_);

  nav2_util::declare_parameter_if_not_declared(node, "max_acc",
                                               rclcpp::ParameterValue(0.75));
  node->get_parameter("max_acc", max_acc_);
  // 1.5/2.0=0.75

  nav2_util::declare_parameter_if_not_declared(node, "min_acc",
                                               rclcpp::ParameterValue(-0.75));
  node->get_parameter("min_acc", min_acc_);

  nav2_util::declare_parameter_if_not_declared(node, "max_translational_vel",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter("max_translational_vel", max_translational_vel_);

  nav2_util::declare_parameter_if_not_declared(node, "min_translational_vel",
                                               rclcpp::ParameterValue(-0.5));
  node->get_parameter("min_translational_vel", min_translational_vel_);

  nav2_util::declare_parameter_if_not_declared(node, "min_approach_velocity",
                                               rclcpp::ParameterValue(0.05));
  node->get_parameter("min_approach_velocity", min_approach_velocity_);

  marker_pub_ =
      node->template create_publisher<visualization_msgs::msg::Marker>(
          "diff_drive_behaviors_visualization_marker", 1);
}

nav2_behaviors::Status Linear::onRun(
    const std::shared_ptr<const LinearAction::Goal> command) {
  command_x_ = command->target_x;
  command_speed_ = std::fabs(command->control_speed);
  tolerance_ = std::fabs(command->tolerance);
  velocity_profile_ = command->velocity_profile;
  command_time_allowance_ = command->time_allowance;
  deadline_time_ = this->steady_clock_.now() + command_time_allowance_;
  min_approach_velocity_ = std::fabs(min_approach_velocity_);

  if (command_time_allowance_.seconds() <= 0.0) {
    RCLCPP_INFO(this->logger_,
                "Invalid(negative or zero) allowed time goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_x_ == 0.0) {
    RCLCPP_INFO(this->logger_, "Invalid(zero) command_x goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (tolerance_ >= std::fabs(command_x_)) {
    RCLCPP_INFO(this->logger_,
                "Invalid(within tolerance) command_x goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_speed_ == 0.0) {
    RCLCPP_INFO(this->logger_, "Invalid(zero) command_speed goal received");
    return nav2_behaviors::Status::FAILED;
  } else if (command_speed_ < min_approach_velocity_) {
    RCLCPP_INFO(this->logger_,
                "command_speed is lower than min_approach_velocity");
    return nav2_behaviors::Status::FAILED;
  } else if (command_x_ > 0.0) {
    if (command_x_ > std::fabs(allowable_forward_travel_distance_)) {
      RCLCPP_INFO(this->logger_,
                  "Target x is more than allowable forward travel distance");
      return nav2_behaviors::Status::FAILED;
    } else if (command_speed_ > std::fabs(max_translational_vel_)) {
      RCLCPP_INFO(this->logger_,
                  "Command velocity is more than allowable max velocity");
      return nav2_behaviors::Status::FAILED;
    }
  } else if (command_x_ < 0.0) {
    if (std::fabs(command_x_) >
        std::fabs(allowable_backward_travel_distance_)) {
      RCLCPP_INFO(this->logger_,
                  "Target x is more than allowable backward travel distance");
      return nav2_behaviors::Status::FAILED;
    } else if (command_speed_ > std::fabs(min_translational_vel_)) {
      RCLCPP_INFO(this->logger_,
                  "Command velocity is less than allowable min velocity");
      return nav2_behaviors::Status::FAILED;
    }
  }

  if (!nav2_util::getCurrentPose(initial_pose_, *this->tf_, this->global_frame_,
                                 this->robot_base_frame_,
                                 this->transform_tolerance_)) {
    RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  double initial_yaw = tf2::getYaw(initial_pose_.pose.orientation);

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
  marker_.ns = "move_linear";
  marker_.id = 15000;
  marker_.pose = initial_pose_.pose;
  marker_.pose.position.x =
      initial_pose_.pose.position.x + command_x_ * cos(initial_yaw);
  marker_.pose.position.y =
      initial_pose_.pose.position.y + command_x_ * sin(initial_yaw);
  marker_.lifetime = command_time_allowance_;

  marker_pub_->publish(marker_);

  marker_pub_->on_deactivate();

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status Linear::onCycleUpdate() {
  rclcpp::Duration time_remaining = deadline_time_ - this->steady_clock_.now();
  if (time_remaining.seconds() < 0.0) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_,
                "Exceeded time allowance before reaching the Linear goal - "
                "Exiting Linear");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *this->tf_, this->global_frame_,
                                 this->robot_base_frame_,
                                 this->transform_tolerance_)) {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);
  double distance_traveled = copysign(distance, command_x_);
  double distance_remaining = command_x_ - copysign(distance, command_x_);

  feedback_->distance_traveled = distance_traveled;
  feedback_->distance_remaining = distance_remaining;
  this->action_server_->publish_feedback(feedback_);

  if (command_x_ > 0.0) {
    if (distance_remaining < tolerance_) {
      this->stopRobot();
      return nav2_behaviors::Status::SUCCEEDED;
    }
  } else if (command_x_ < 0.0) {
    if (distance_remaining > -tolerance_) {
      this->stopRobot();
      return nav2_behaviors::Status::SUCCEEDED;
    }
  }

  double speed;
  switch (velocity_profile_) {
    case 0:
      speed = command_speed_;
      break;
    case 1:
      speed = std::max(
          std::min(std::fabs(feedback_->distance_remaining / command_x_) *
                       command_speed_,
                   command_speed_),
          min_approach_velocity_);
      break;
    case 2:
      speed = std::max(
          std::min(sqrt(std::fabs(feedback_->distance_remaining / command_x_)) *
                       command_speed_,
                   command_speed_),
          min_approach_velocity_);
      break;
    default:
      speed = command_speed_;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  if (command_x_ > 0)
    cmd_vel->linear.x =
        std::min(std::fabs(speed), std::fabs(max_translational_vel_));
  else
    cmd_vel->linear.x =
        std::max(-std::fabs(speed), -std::fabs(min_translational_vel_));

  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  // if (!isCollisionFree(distance_remaining, cmd_vel.get(), pose2d)) {
  //   this->stopRobot();
  //   RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting Linear");
  //   return nav2_behaviors::Status::FAILED;
  // }

  this->vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

bool Linear::isCollisionFree(const double& distance_remaining,
                             geometry_msgs::msg::Twist* cmd_vel,
                             geometry_msgs::msg::Pose2D& pose2d) {
  // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  // const double diff_dist = abs(command_x_) - distance;
  const int max_cycle_count =
      static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change =
        cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
    pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
    cycle_count++;

    if (std::fabs(distance_remaining) - abs(sim_position_change) <= 0.) {
      break;
    }

    if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}  // namespace diff_drive_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_drive_behaviors::Linear, nav2_core::Behavior)