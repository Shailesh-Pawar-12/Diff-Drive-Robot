#include "behaviortree_ros2/bt_action_node.hpp"
#include "bt_interfaces/action/linear.hpp"
#include "bt_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace BT;

class LinearAction : public RosActionNode<bt_interfaces::action::Linear> {
 public:
  LinearAction(const std::string& name, const NodeConfig& conf,
               const RosNodeParams& params)
      : RosActionNode<bt_interfaces::action::Linear>(name, conf, params) {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>(
        "bt_action_node");
  }

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {InputPort<double>("target_x", "Spin distance"),
         InputPort<double>("control_speed", "Allowed time for spinning"),
         InputPort<double>("tolerance", "Allowed time for spinning"),
         InputPort<int>("velocity_profile", "Allowed time for spinning"),
         InputPort<double>("time_allowance", "Allowed time for spinning")});
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
