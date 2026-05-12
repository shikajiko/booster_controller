#include "booster_joint_manager/node/joint_manager_node.hpp"

#include <chrono>

namespace booster_joint_manager
{

JointManagerNode::JointManagerNode(const rclcpp::Node::SharedPtr & node) : node(node)
{
  //connection to booster's motor via SDK
  joint_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
      update_low_state(msg);
      publish_joint_state();
    });
  
  joint_cmd_publisher =
    node->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 10);

  //connection to other nodes via custom topic
  joint_state_publisher =
    node->create_publisher<booster_interface::msg::LowState>("joint/joint_states", 10);

  set_cmd_subscriber = node->create_subscription<booster_joint_interface::msg::SetJoints>(
    "joint/set_joints",
    10,
    [this](booster_joint_interface::msg::SetJoints::SharedPtr msg) {
      auto targets = joint_msg_to_target(*msg);
      joint_manager.handle_set_joints(targets);
    });

  command_timer = node->create_wall_timer(
    std::chrono::milliseconds(kCommandTickMs),
    [this](){
      booster_interface::msg::LowCmd cmd;
      if (joint_manager.tick_command(cmd)) {
        publish_joint_cmd(cmd);
      }
    });
}

void JointManagerNode::update_low_state(const booster_interface::msg::LowState::SharedPtr & msg)
{
  joint_manager.update_low_state(*msg);
}

bool JointManagerNode::get_joint_state(
  b1::JointIndex joint,
  booster_interface::msg::MotorState & state) const
{
  return joint_manager.get_joint_state(joint, state);
}

void JointManagerNode::print_joint_info(b1::JointIndex joint)
{
  booster_interface::msg::MotorState state;
  if (!get_joint_state(joint, state)) {
    RCLCPP_WARN(
      node->get_logger(),
      "No /low_state data received yet for joint %d (%.*s).",
      static_cast<int>(joint),
      static_cast<int>(joint_name(joint).size()),
      joint_name(joint).data());
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "joint %d (%.*s): q=%.4f dq=%.4f ddq=%.4f tau_est=%.4f",
    static_cast<int>(joint),
    static_cast<int>(joint_name(joint).size()),
    joint_name(joint).data(),
    state.q,
    state.dq,
    state.ddq,
    state.tau_est);
}

void JointManagerNode::print_all_joint_info()
{
  for (const auto joint : kAllJoints) {
    print_joint_info(joint);
  }
}

void JointManagerNode::publish_joint_cmd(const booster_interface::msg::LowCmd & cmd)
{
  joint_cmd_publisher->publish(cmd);
}

void JointManagerNode::publish_joint_state()
{
  booster_interface::msg::LowState state;
  if (!joint_manager.get_low_state(state)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No joint data received from the motor"
    );
    return;
  }

  joint_state_publisher->publish(state);
}

std::vector<JointCommandTarget> JointManagerNode::joint_msg_to_target(
  const booster_joint_interface::msg::SetJoints & msg)
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(msg.joints.size());

  for (const auto & joint : msg.joints) {
    if (joint.id >= b1::kJointCnt) {
      continue;
    }

    targets.push_back(
      JointCommandTarget{
        static_cast<b1::JointIndex>(joint.id),
        joint.position,
        joint.velocity,
      });
  }

  return targets;
}
}  // namespace booster_joint_manager
