#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "diff_drive_behavior_tree/linear_action.hpp"
#include "diff_drive_behavior_tree/rotate_action.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace BT;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_task_node");

  nh->declare_parameter(
      "task_bt_file",
      "/home/shailesh/Documents/path_planner/diff_drive_ws/src/"
      "diff_drive_behavior_tree/behavior_trees/linear_rotate_action.xml");

  Blackboard::Ptr blackboard_;
  blackboard_ = Blackboard::create();
  blackboard_->set<rclcpp::Node::SharedPtr>("bt_action_node",
                                            nh->shared_from_this());
  blackboard_->set<double>("target_yaw", 0.2);
  blackboard_->set<double>("control_angular_speed", 0.5);
  blackboard_->set<double>("tolerance", 0.05);
  blackboard_->set<int>("velocity_profile", 2);
  blackboard_->set<double>("time_allowance", 30);
  blackboard_->set<double>("target_x", 1.0);
  blackboard_->set<double>("control_speed", 0.5);

  BehaviorTreeFactory factory;

  RosNodeParams rotate_params;
  rotate_params.nh = nh;
  rotate_params.default_port_value = "rotate";
  factory.registerNodeType<RotateAction>("RotateAction", rotate_params);

  RosNodeParams linear_params;
  linear_params.nh = nh;
  linear_params.default_port_value = "linear";
  factory.registerNodeType<LinearAction>("LinearAction", linear_params);

  BT::Tree task_tree_;
  task_tree_ = factory.createTreeFromFile(
      nh->get_parameter("task_bt_file").as_string(), blackboard_);

  task_tree_.tickWhileRunning();

  return 0;
}
