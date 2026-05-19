#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/joint_manager/joint_manager.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "booster_joint_interface/msg/set_torques.hpp"
#include "booster_joint_interface/srv/prepare_transition.hpp"
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
  using JointPrepareService = booster_joint_interface::srv::PrepareTransition;
  using NextMode = booster_joint_interface::msg::TransitionCommand;

  rclcpp::Node::SharedPtr node;
  JointManager joint_manager;

  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr joint_cmd_publisher;
  rclcpp::Publisher<booster_interface::msg::LowState>::SharedPtr joint_state_publisher;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr joint_state_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetJoints>::SharedPtr set_cmd_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetTorques>::SharedPtr set_torques_subscriber;
  rclcpp::Service<JointPrepareService>::SharedPtr joint_prepare_service;
  rclcpp::TimerBase::SharedPtr command_timer;

  void publish_joint_state();
  void publish_joint_cmd(const booster_interface::msg::LowCmd & cmd);
  void prepare_mode_switch();
  void print_target_command(const std::vector<JointCommandTarget> & targets);
  void update_low_state(const booster_interface::msg::LowState::SharedPtr & msg);
  void handle_prepare_transition_request(const std::shared_ptr<JointPrepareService::Request> req, std::shared_ptr<JointPrepareService::Response> res);
  void handle_mode_prepare(uint8_t target_mode, std::shared_ptr<JointPrepareService::Response> res);
  void handle_upper_body_prepare(bool enable, std::shared_ptr<JointPrepareService::Response> res);


  std::vector<JointCommandTarget> joint_msg_to_target(const booster_joint_interface::msg::SetJoints & msg);
  std::vector<JointIndex> id_to_joint_index(const std::vector<uint8_t> & ids);
};

}  // namespace booster_joint_manager
