#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/joint_manager/joint_manager.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_joint_manager
{

class JointManagerNode
{
public:
  explicit JointManagerNode(const rclcpp::Node::SharedPtr & node);

  void update_low_state(const booster_interface::msg::LowState::SharedPtr & msg);
  bool get_joint_state(b1::JointIndex joint, booster_interface::msg::MotorState & state) const;
  void print_joint_info(b1::JointIndex joint);
  void print_all_joint_info();
  void publish_joint_cmd(const booster_interface::msg::LowCmd & cmd);

private:
  rclcpp::Node::SharedPtr node;
  JointManager joint_manager;

  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr joint_cmd_publisher;
  rclcpp::Publisher<booster_interface::msg::LowState>::SharedPtr joint_state_publisher;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr joint_state_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetJoints>::SharedPtr set_cmd_subscriber;
  rclcpp::TimerBase::SharedPtr command_timer;

  void publish_joint_state();
  std::vector<JointCommandTarget> joint_msg_to_target(const booster_joint_interface::msg::SetJoints & msg);
};

}  // namespace booster_joint_manager
