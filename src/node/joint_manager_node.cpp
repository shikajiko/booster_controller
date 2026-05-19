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

  joint_prepare_service = node->create_service<JointPrepareService>(
    "prep_transition_service",
    std::bind(
      &JointManagerNode::handle_prepare_transition_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

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
  JointIndex joint,
  booster_interface::msg::MotorState & state) const
{
  return joint_manager.get_joint_state(joint, state);
}

void JointManagerNode::print_joint_info(JointIndex joint)
{
  booster_interface::msg::MotorState state;
  if (!get_joint_state(joint, state)) {
    RCLCPP_WARN(
      node->get_logger(),
      "joint %02d (%.*s): q unavailable",
      static_cast<int>(joint),
      static_cast<int>(joint_name(joint).size()),
      joint_name(joint).data());
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "joint %02d (%.*s): q=% .4f",
    static_cast<int>(joint),
    static_cast<int>(joint_name(joint).size()),
    joint_name(joint).data(),
    state.q);
}

void JointManagerNode::print_all_joint_info()
{
  RCLCPP_INFO(node->get_logger(), "Current joint q values:");
  for (const auto joint : kAllJoints) {
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
      static_cast<int>(joint_name(target.joint).size()),
      joint_name(target.joint).data(),
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
  RCLCPP_DEBUG(
    node->get_logger(),
    "Published joint/joint_states with %zu motor states",
    state.motor_state_serial.size());
}

void JointManagerNode::handle_prepare_transition_request(const std::shared_ptr<JointPrepareService::Request> req,
    std::shared_ptr<JointPrepareService::Response> res)
{
  const auto & command = req->command;
  RCLCPP_INFO(
    node->get_logger(),
    "Received prep_transition_service request: transition=%u",
    static_cast<unsigned int>(command.transition));
    
  switch (command.transition) {
    case booster_joint_interface::msg::TransitionCommand::TRANSITION_MODE_SWITCH:
      handle_mode_prepare(command.target_mode, res);
      RCLCPP_INFO(
        node->get_logger(),
        "Handled mode transition prepare request: target_mode=%u, success=%s, message='%s'",
        static_cast<unsigned int>(command.target_mode),
        res->success ? "true" : "false",
        res->message.c_str());
      return;

    case booster_joint_interface::msg::TransitionCommand::TRANSITION_UPPER_BODY_CONTROL:
      handle_upper_body_prepare(command.upper_body_enable, res);
      RCLCPP_INFO(
        node->get_logger(),
        "Handled upper body transition prepare request: enable=%s, success=%s, message='%s'",
        command.upper_body_enable ? "true" : "false",
        res->success ? "true" : "false",
        res->message.c_str());
      return;

    default:
      res->success = false;
      res->message = "Unknown transition type";
      RCLCPP_WARN(
        node->get_logger(),
        "Rejected prep_transition_service request: unknown transition=%u",
        static_cast<unsigned int>(command.transition));
      return;
  }
}

void JointManagerNode::handle_mode_prepare(uint8_t target_mode, std::shared_ptr<JointPrepareService::Response> res)
{
  switch (target_mode) {
    case NextMode::MODE_DAMPING:
      res->success = true;
      return;

    case NextMode::MODE_STAND:
    case NextMode::MODE_WALK:
      prepare_mode_switch();
      joint_manager.set_init_position(false);
      print_target_command(joint_manager.get_target_command());
      res->success = true;
      res->message = "Preparing init pose";
      return;

    case NextMode::MODE_CUSTOM:
      prepare_mode_switch();
      joint_manager.maintain_current_pose();
      print_target_command(joint_manager.get_target_command());
      res->success = true;
      res->message = "Preparing current-pose hold";
      return;

    default:
      res->success = false;
      res->message = "Unknown target mode";
      return;
  }
}

void JointManagerNode::prepare_mode_switch()
{
  print_all_joint_info();
}

void JointManagerNode::handle_upper_body_prepare(bool enable, std::shared_ptr<JointPrepareService::Response> res)
{
  if (enable) {
    joint_manager.maintain_current_pose();
    res->success = true;
    res->message = "Prepared upper-body current-pose hold";
    return;
  }

  joint_manager.set_init_position(true);
  res->success = true;
  res->message = "Prepared upper-body init pose";
}

std::vector<JointCommandTarget> JointManagerNode::joint_msg_to_target(
  const booster_joint_interface::msg::SetJoints & msg)
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(msg.joints.size());

  for (const auto & joint : msg.joints) {
    if (joint.id >= kJointCnt) {
      continue;
    }

    targets.push_back(
      JointCommandTarget{
        static_cast<JointIndex>(joint.id),
        joint.position,
        joint.velocity,
      });
  }

  return targets;
}

std::vector<JointIndex> JointManagerNode::id_to_joint_index(
  const std::vector<uint8_t> & ids)
{
  std::vector<JointIndex> joints;
  joints.reserve(ids.size());

  for (const auto id : ids) {
    if (id >= kJointCnt) {
      continue;
    }
    joints.push_back(static_cast<JointIndex>(id));
  }

  return joints;
}
}  // namespace booster_joint_manager
