#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ekf/ekf.hpp" 

using namespace std::chrono_literals;
using namespace Eigen;
using namespace rclcpp;
using namespace geometry_msgs::msg;

class EKFNode : public rclcpp::Node {
 public:
  EKFNode()
      : Node("ekf_node"),
        ekf_(std::make_unique<
             DifferentialDriveEKF>()) { 

    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    // Create the EKF
    RCLCPP_INFO(this->get_logger(), "EKF Node Started (DifferentialDriveEKF).");

    // Subscribers
    control_sub_ = this->create_subscription<Twist>(
        "/diff_drive_base_controller/cmd_vel_unstamped", 10,
        std::bind(&EKFNode::control_callback, this, std::placeholders::_1));

    // Publisher for the EKF state
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom", 10);

    // Initialize EKF (example for DifferentialDriveEKF)
    VectorXd initial_state(
        5);  // [x, y, theta, v, omega] (forward velocity and angular velocity)
    initial_state << 0, 0, 0, 0, 0;  // Initial state vector
    MatrixXd initial_cov =
        MatrixXd::Identity(5, 5);  // Initial covariance matrix
    ekf_->Init(initial_state, initial_cov);

    // Set the loop rate
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&EKFNode::timer_callback, this));
  }

 private:
  void control_callback(const Twist::SharedPtr msg) {
    // Extract control inputs from the Twist message for DifferentialDriveEKF
    VectorXd control_input(2);
    control_input << msg->linear.x,
        msg->angular.z;  // Linear velocity and angular velocity
    std::cout << "msg->linear.x = " << msg->linear.x << " | "
              << "msg->angular.z = " << msg->angular.z << std::endl;

    rclcpp::Time now = this->get_clock()->now();  // Get the current time
    double current_seconds =
        now.seconds();  // Get the seconds as a floating-point number

    if (!initial_cmd_vel_rec) {
      prev_seconds = current_seconds;
      initial_cmd_vel_rec = true;
    }

    double delta_t = current_seconds - prev_seconds;

    // Run the EKF prediction with the control inputs
    ekf_->Predict(delta_t, control_input);  // Assuming 0.2s as the time step
    prev_seconds = current_seconds;
  }

  void timer_callback() {
    // Get the current state from the EKF
    VectorXd state = ekf_->GetState();

    // Publish the estimated state (x, y, theta)
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";

    odom_msg.child_frame_id = "chassis";

    std::cout << "state(0) = " << state(0) << std::endl;

    odom_msg.pose.pose.position.x = state(0);
    odom_msg.pose.pose.position.y = state(1);
    odom_msg.pose.pose.orientation.z = sin(state(2) / 2);
    odom_msg.pose.pose.orientation.w = cos(state(2) / 2);

    odom_msg.twist.twist.linear.x = state(3);   // Forward velocity
    odom_msg.twist.twist.angular.z = state(4);  // Angular velocity

    odom_pub_->publish(odom_msg);
  }

  rclcpp::Subscription<Twist>::SharedPtr control_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<EKF> ekf_;  // The EKF object
  bool initial_cmd_vel_rec = false;
  double prev_seconds = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}