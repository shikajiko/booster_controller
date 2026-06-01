#include "booster_controller/node/controller_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

namespace booster_joint_manager
{

JointManagerNode::JointManagerNode(const rclcpp::Node::SharedPtr & node) : node(node)
{
  //connection to booster's motor via SDK
  joint_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
      current_joint_states = msg->motor_state_parallel;
      update_joint_state(msg->motor_state_parallel);
    });

  joint_cmd_publisher =
    node->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 10);

  set_cmd_subscriber = node->create_subscription<booster_joint_interface::msg::SetJoints>(
    "joint/set_joints",
    10,
    [this](booster_joint_interface::msg::SetJoints::SharedPtr msg) {
      RCLCPP_INFO(
        this->node->get_logger(),
        "Received joint/set_joints publish request with %zu joints",
        msg->joints.size());
      auto targets = joint_msg_to_target(*msg);
      joint_manager.handle_set_joints(targets);
    });

  set_torques_subscriber = node->create_subscription<booster_joint_interface::msg::SetTorques>(
    "joint/set_torques",
    10,
    [this](booster_joint_interface::msg::SetTorques::SharedPtr msg) {
      RCLCPP_INFO(
        this->node->get_logger(),
        "Received joint/set_torques publish request with %zu joints, torque %s",
        msg->ids.size(),
        msg->torque_enable ? "enable" : "disable");
      auto joints = id_to_joint_index(msg->ids);
      joint_manager.handle_set_torques(joints, msg->torque_enable);
    }
    );

  joint_transition_handler = std::make_unique<JointTransitionHandler>(node, joint_manager);
  trajectory_controller = std::make_unique<TrajectoryController>();
  trajectory_controller->init(node);

  command_timer = node->create_wall_timer(
    std::chrono::milliseconds(Joint::kCommandFrequencyMs),
    [this](){
      booster_interface::msg::LowCmd cmd;
      if (trajectory_controller && trajectory_controller->has_work()) {
        std::vector<double> current_joint_q;
        current_joint_q.reserve(current_joint_states.size());
        for (const auto& state : current_joint_states) {
          current_joint_q.push_back(state.q);
        }

        trajectory_controller->activate();
        trajectory_controller->update(
          Joint::kControlDt,
          current_joint_q,
          &cmd);

        publish_joint_cmd(cmd);

        if (!trajectory_controller->has_work()) {
          trajectory_controller->deactivate();
        }
        
      } else if (joint_manager.tick_command(cmd)) {
        publish_joint_cmd(cmd);
      }
    });
}

void JointManagerNode::update_joint_state(const std::vector<booster_interface::msg::MotorState> & msg)
{
  joint_manager.update_joint_state(msg);
}

bool JointManagerNode::get_joint_state(
  Joint::JointIndex joint,
  booster_interface::msg::MotorState & state) const
{
  return joint_manager.get_joint_state(joint, state);
}

void JointManagerNode::print_joint_info(Joint::JointIndex joint)
{
  booster_interface::msg::MotorState state;
  if (!get_joint_state(joint, state)) {
    RCLCPP_WARN(
      node->get_logger(),
      "joint %02d (%.*s): q unavailable",
      static_cast<int>(joint),
      static_cast<int>(Joint::joint_name(joint).size()),
      Joint::joint_name(joint).data());
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "joint %02d (%.*s): q=% .4f",
    static_cast<int>(joint),
    static_cast<int>(Joint::joint_name(joint).size()),
    Joint::joint_name(joint).data(),
    state.q);
}

void JointManagerNode::print_all_joint_info()
{
  RCLCPP_INFO(node->get_logger(), "Current joint q values:");
  for (const auto joint : Joint::kAllJoints) {
    print_joint_info(joint);
  }
}

void JointManagerNode::print_target_command(const std::vector<JointCommandTarget> & targets)
{
  RCLCPP_INFO(node->get_logger(), "Current target_command positions:");
  for (const auto & target : targets) {
    RCLCPP_INFO(
      node->get_logger(),
      "target joint %02d (%.*s): q=% .4f velocity=% .4f weight=% .4f",
      static_cast<int>(target.joint),
      static_cast<int>(Joint::joint_name(target.joint).size()),
      Joint::joint_name(target.joint).data(),
      target.position,
      target.velocity,
      target.weight);
  }
}

void JointManagerNode::publish_joint_cmd(const booster_interface::msg::LowCmd & cmd)
{
  RCLCPP_DEBUG(node->get_logger(), "Publishing /joint_ctrl command");
  joint_cmd_publisher->publish(cmd);
}

std::vector<JointCommandTarget> JointManagerNode::joint_msg_to_target(
  const booster_joint_interface::msg::SetJoints & msg)
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(msg.joints.size());

  for (const auto & joint : msg.joints) {
    if (joint.id >= Joint::kJointCnt) {
      continue;
    }
    const auto target_pos = std::clamp(
      joint.position, Joint::kMinJointLimit[joint.id], Joint::kMaxJointLimit[joint.id]);

    targets.push_back(
      JointCommandTarget{
        static_cast<Joint::JointIndex>(joint.id),
        target_pos,
        joint.velocity,
        0.5,
      });
  }

  return targets;
}

std::vector<Joint::JointIndex> JointManagerNode::id_to_joint_index(
  const std::vector<uint8_t> & ids)
{
  std::vector<Joint::JointIndex> joints;
  joints.reserve(ids.size());

  for (const auto id : ids) {
    if (id >= Joint::kJointCnt) {
      continue;
    }
    joints.push_back(static_cast<Joint::JointIndex>(id));
  }

  return joints;
}
}  // namespace booster_joint_manager
