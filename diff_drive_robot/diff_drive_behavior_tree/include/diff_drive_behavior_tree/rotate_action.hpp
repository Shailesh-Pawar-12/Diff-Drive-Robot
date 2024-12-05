#include "behaviortree_ros2/bt_action_node.hpp"
#include "bt_interfaces/action/rotate.hpp"
#include "bt_macros.hpp"

using namespace BT;

class RotateAction : public RosActionNode<bt_interfaces::action::Rotate> {
 public:
  RotateAction(const std::string& name, const NodeConfig& conf,
               const RosNodeParams& params)
      : RosActionNode<bt_interfaces::action::Rotate>(name, conf, params) {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>(
        "bt_action_node");
  }

  static BT::PortsList providedPorts() {
    return providedBasicPorts({InputPort<double>("target_yaw"),
                               InputPort<double>("control_angular_speed"),
                               InputPort<double>("tolerance"),
                               InputPort<int>("velocity_profile"),
                               InputPort<double>("time_allowance")});
  }

  bool setGoal(Goal& goal) override;

  // void onHalt() override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;

  void on_pause(Goal& goal, const std::shared_ptr<const Feedback> feedback);

 protected:
  rclcpp::Node::SharedPtr node_;
};
