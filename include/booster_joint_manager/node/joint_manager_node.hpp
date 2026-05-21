#pragma once

#include <memory>
#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/joint_manager/joint_manager.hpp"
#include "booster_joint_manager/node/joint_transition_handler.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "booster_joint_interface/msg/set_torques.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_joint_manager
{

class JointManagerNode
{
public:
  explicit JointManagerNode(const rclcpp::Node::SharedPtr & node);

  bool get_joint_state(JointIndex joint, booster_interface::msg::MotorState & state) const;
  void print_joint_info(JointIndex joint);
  void print_all_joint_info();

private:
  rclcpp::Node::SharedPtr node;
  JointManager joint_manager;
  std::unique_ptr<JointTransitionHandler> joint_transition_handler;

  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr joint_cmd_publisher;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr joint_state_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetJoints>::SharedPtr set_cmd_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetTorques>::SharedPtr set_torques_subscriber;
  rclcpp::TimerBase::SharedPtr command_timer;
  rclcpp::TimerBase::SharedPtr transition_timer;

  void publish_joint_cmd(const booster_interface::msg::LowCmd & cmd);
  void print_target_command(const std::vector<JointCommandTarget> & targets);
  void update_joint_state(const std::vector<booster_interface::msg::MotorState> & msg);

  std::vector<JointCommandTarget> joint_msg_to_target(const booster_joint_interface::msg::SetJoints & msg);
  std::vector<JointIndex> id_to_joint_index(const std::vector<uint8_t> & ids);
};

}  // namespace booster_joint_manager
