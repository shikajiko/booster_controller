#pragma once

#include <atomic>
#include <memory>

#include "booster_joint_interface/srv/prepare_transition.hpp"
#include "booster_joint_interface/msg/transition_command.hpp"
#include "booster_joint_manager/joint_manager/joint_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_joint_manager
{
class JointTransitionHandler
{
public:
JointTransitionHandler(
  const rclcpp::Node::SharedPtr & node,
  JointManager & joint_manager);

private:
using JointPrepareService = booster_joint_interface::srv::PrepareTransition;
using TransitionCommand = booster_joint_interface::msg::TransitionCommand;

rclcpp::Node::SharedPtr node;
JointManager & joint_manager;
rclcpp::Service<JointPrepareService>::SharedPtr joint_prepare_service;
std::atomic_bool mode_prepare_requested{false};

void parse_prepare_transition_request(
  const std::shared_ptr<JointPrepareService::Request> req,
  std::shared_ptr<JointPrepareService::Response> res);
bool handle_mode_switch(uint8_t target_mode, float & delay_second);
bool handle_upper_control_switch(bool enable, float & delay_second);
};

}  // namespace booster_joint_manager
