#pragma once

#include <memory>
#include <vector>

#include "action_interface/action/action_trajectory.hpp"
#include "booster_controller/controller_manager/controller_manager.hpp"
#include "booster_controller/joint_state_manager/joint_state_manager.hpp"
#include "booster_controller/utils/joint_command.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "booster_joint_interface/msg/set_torques.hpp"
#include "booster_joint_interface/srv/prepare_transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace booster_controller
{

class ControllerManagerNode
{
public:
  using PrepareTransition = booster_joint_interface::srv::PrepareTransition;
  using TrajectoryAction = action_interface::action::ActionTrajectory;
  using TrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<TrajectoryAction>;

  explicit ControllerManagerNode(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node;

  JointStateManager joint_state_manager;
  ControllerManager controller_manager;

  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr joint_state_subscriber;
  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr joint_command_publisher;
  rclcpp::Subscription<booster_joint_interface::msg::SetJoints>::SharedPtr set_joints_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetTorques>::SharedPtr set_torques_subscriber;
  rclcpp::Service<PrepareTransition>::SharedPtr prepare_transition_service;
  rclcpp_action::Server<TrajectoryAction>::SharedPtr trajectory_action_server;
  rclcpp::TimerBase::SharedPtr timer;

  void handle_joint_state(booster_interface::msg::LowState::SharedPtr msg);
  void handle_set_joints(booster_joint_interface::msg::SetJoints::SharedPtr msg);
  void handle_set_torques(booster_joint_interface::msg::SetTorques::SharedPtr msg);
  void handle_prepare_transition(
    const std::shared_ptr<PrepareTransition::Request> request,
    std::shared_ptr<PrepareTransition::Response> response);

  rclcpp_action::GoalResponse handle_trajectory_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TrajectoryAction::Goal> goal);
  rclcpp_action::CancelResponse handle_trajectory_cancel(
    std::shared_ptr<TrajectoryGoalHandle> goal_handle);

  void handle_trajectory_accepted(std::shared_ptr<TrajectoryGoalHandle> goal_handle);
  void tick();
  
  std::vector<JointCommandTarget> msg_to_targets(const booster_joint_interface::msg::SetJoints& msg) const;
  std::vector<Joint::JointIndex> id_to_joint_idx( const std::vector<uint8_t>& ids) const;
};

}  // namespace booster_controller