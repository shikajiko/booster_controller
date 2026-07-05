#pragma once

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "booster_action_interface/action/action_trajectory.hpp"
#include "booster_controller/controller/mode_transition_controller.hpp"
#include "booster_controller/controller/set_joint_controller.hpp"
#include "booster_controller/controller/torque_controller.hpp"
#include "booster_controller/controller/trajectory_controller.hpp"
#include "booster_controller/joint_state_manager/joint_state_manager.hpp"
#include "booster_controller/utils/joint_command.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_interface/msg/transition_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace booster_controller
{

class ControllerManager
{
public:
  using TrajectoryAction = booster_action_interface::action::ActionTrajectory;
  using TrajectoryGoalHandle =
    rclcpp_action::ServerGoalHandle<TrajectoryAction>;
  using TransitionCommand =
    booster_joint_interface::msg::TransitionCommand;

  enum class ActiveController
  {
    none,
    mode_transition,
    trajectory,
    torque,
    set_joint
  };

  explicit ControllerManager(
    rclcpp::Node::SharedPtr node,
    JointStateManager& joint_state_manager);

  std::optional<booster_interface::msg::LowCmd> update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& states);

  bool submit_trajectory(
    std::shared_ptr<TrajectoryGoalHandle> goal_handle);

  bool cancel_trajectory(
    std::shared_ptr<TrajectoryGoalHandle> goal_handle);

  bool submit_joints(
    const std::vector<JointCommandTarget>& targets);

  bool submit_torques(
    const std::vector<Joint::JointIndex>& joints,
    bool enable);

  bool submit_mode_transition(
    const TransitionCommand& command,
    float& out_delay);

  bool trajectory_busy() const;

  bool cancel_pending_trajectory(
    std::shared_ptr<TrajectoryGoalHandle> goal_handle);

  void abort_pending_trajectory(
    std::string_view reason);

  static std::string_view controller_name(
    ActiveController controller);

private:
  JointStateManager& joint_state_manager;

  TrajectoryController trajectory_controller;
  SetJointController set_joint_controller;
  TorqueController torque_controller;
  ModeTransitionController mode_transition_controller;

  ActiveController active_controller =
    ActiveController::none;

  std::shared_ptr<TrajectoryGoalHandle>
    pending_trajectory_goal;

  ActiveController select_active_controller() const;

  IController* controller_for(
    ActiveController controller);

  void deactivate_lower_priority_controllers(
    ActiveController controller);

  bool start_pending_trajectory_goal();

  rclcpp::Logger logger;
};

}  // namespace booster_controller