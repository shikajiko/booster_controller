#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "booster_controller/joint.hpp"
#include "booster_controller/joint_manager/joint_manager.hpp"
#include "booster_controller/mode_transition_controller.hpp"
#include "booster_controller/set_joint_controller.hpp"
#include "booster_controller/torque_controller.hpp"
#include "booster_controller/trajectory_controller.hpp"
#include "action_interface/action/action_trajectory.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_interface/msg/set_joints.hpp"
#include "booster_joint_interface/msg/set_torques.hpp"
#include "booster_joint_interface/srv/prepare_transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace booster_joint_manager
{

class ControllerManagerNode {
public:
  explicit ControllerManagerNode(const rclcpp::Node::SharedPtr& node);

  bool get_joint_state(Joint::JointIndex joint, booster_interface::msg::MotorState& state) const;
  void print_joint_info(Joint::JointIndex joint);
  void print_all_joint_info();

private:
  enum class ActiveController {
    none,
    mode_transition,
    trajectory,
    torque,
    set_joint
  };

  using PrepareTransition = booster_joint_interface::srv::PrepareTransition;
  using TransitionCommand = booster_joint_interface::msg::TransitionCommand;
  using TrajectoryAction = action_interface::action::ActionTrajectory;
  using TrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<TrajectoryAction>;

  rclcpp::Node::SharedPtr node;
  JointManager joint_manager;
  ModeTransitionController mode_transition_controller;
  TrajectoryController trajectory_controller;
  TorqueController torque_controller;
  SetJointController set_joint_controller;
  ActiveController active_controller = ActiveController::none;

  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr joint_cmd_publisher;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr joint_state_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetJoints>::SharedPtr set_cmd_subscriber;
  rclcpp::Subscription<booster_joint_interface::msg::SetTorques>::SharedPtr set_torques_subscriber;
  rclcpp::Service<PrepareTransition>::SharedPtr prepare_transition_service;
  rclcpp_action::Server<TrajectoryAction>::SharedPtr trajectory_action_server;
  std::shared_ptr<TrajectoryGoalHandle> pending_trajectory_goal;
  rclcpp::TimerBase::SharedPtr command_timer;

  void publish_joint_cmd(const booster_interface::msg::LowCmd& cmd);
  void update_joint_state(const std::vector<booster_interface::msg::MotorState>& msg);
  void handle_set_joints(const booster_joint_interface::msg::SetJoints& msg);
  void handle_set_torques(const booster_joint_interface::msg::SetTorques& msg);
  void handle_prepare_transition(
    const std::shared_ptr<PrepareTransition::Request> request,
    std::shared_ptr<PrepareTransition::Response> response);
  rclcpp_action::GoalResponse handle_trajectory_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TrajectoryAction::Goal> goal);
  rclcpp_action::CancelResponse handle_trajectory_cancel(
    std::shared_ptr<TrajectoryGoalHandle> goal_handle);
  void handle_trajectory_accepted(std::shared_ptr<TrajectoryGoalHandle> goal_handle);
  void tick_controller();
  ActiveController select_active_controller() const;
  IController* controller_for(ActiveController controller);
  bool start_pending_trajectory_goal();
  void abort_pending_trajectory_goal(std::string_view reason);
  void deactivate_lower_priority_controllers(ActiveController controller);
  static std::string_view controller_name(ActiveController controller);

  std::vector<JointCommandTarget> joint_msg_to_target(
    const booster_joint_interface::msg::SetJoints& msg) const;
  std::vector<Joint::JointIndex> id_to_joint_index(const std::vector<uint8_t>& ids) const;
};

using JointManagerNode = ControllerManagerNode;

}  // namespace booster_joint_manager
