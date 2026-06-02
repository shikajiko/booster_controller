#include "booster_controller/controller_manager/controller_manager.hpp"

namespace booster_controller
{

ControllerManager::ControllerManager( rclcpp::Node::SharedPtr node, JointStateManager& joint_state_manager) : 
  joint_state_manager(joint_state_manager), logger(node->get_logger())
{
  trajectory_controller.init(node);
  mode_transition_controller.init(node);
  set_joint_controller.init(node);
  torque_controller.init(node);
}

std::optional<booster_interface::msg::LowCmd> ControllerManager::update(
  double dt,
  const std::vector<booster_interface::msg::MotorState>& states)
{
  if (!joint_state_manager.has_joint_state()) return std::nullopt;
  const auto next_controller = select_active_controller();

  if (next_controller == ActiveController::none) {
    if (active_controller != ActiveController::none) {
      RCLCPP_INFO(logger, "%.*s controller finished",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data());
      active_controller = ActiveController::none;
    }
    return std::nullopt;
  }

  if (next_controller != active_controller) {
    auto* previous = controller_for(active_controller);
    if (previous && previous->has_work()) {
      previous->deactivate();

      RCLCPP_INFO(logger,
        "%.*s controller deactivated by higher-priority %.*s controller",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data(),
        static_cast<int>(controller_name(next_controller).size()),
        controller_name(next_controller).data());
    }

    active_controller = next_controller;
    auto* activated = controller_for(active_controller);

    if (activated) {
      activated->activate();
      RCLCPP_INFO(logger, "%.*s controller activated",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data());
    }
  }

  if (active_controller == ActiveController::trajectory &&
      !trajectory_controller.has_work())
  {
    if (!start_pending_trajectory_goal()) {
      active_controller = ActiveController::none;
      return std::nullopt;
    }
  }

  auto* controller = controller_for(active_controller);
  if (!controller) return std::nullopt;

  booster_interface::msg::LowCmd cmd;
  controller->update(dt, states, cmd);

  if (!controller->has_work()) {
    controller->deactivate();
    RCLCPP_INFO(logger, "%.*s controller completed",
      static_cast<int>(controller_name(active_controller).size()),
      controller_name(active_controller).data());
    active_controller = ActiveController::none;
  }

  return cmd;
}


bool ControllerManager::submit_trajectory(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  if (trajectory_controller.has_work() || pending_trajectory_goal) {
    RCLCPP_WARN(logger, "Rejected trajectory: already active or queued");
    return false;
  }
  pending_trajectory_goal = goal_handle;
  RCLCPP_INFO(logger, "Trajectory goal queued");
  return true;
}

bool ControllerManager::cancel_trajectory(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  if (pending_trajectory_goal && pending_trajectory_goal == goal_handle) {
    return cancel_pending_trajectory(goal_handle);
  }
  return trajectory_controller.cancel(goal_handle);
}

bool ControllerManager::cancel_pending_trajectory(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  if (!pending_trajectory_goal || pending_trajectory_goal != goal_handle)
    return false;

  auto result = std::make_shared<TrajectoryAction::Result>();
  result->error_code   = TrajectoryAction::Result::ERROR_OK;
  result->error_string = "Cancelled before start";
  pending_trajectory_goal->canceled(result);
  pending_trajectory_goal = nullptr;
  RCLCPP_INFO(logger, "Pending trajectory cancelled before start");
  return true;
}

void ControllerManager::abort_pending_trajectory(std::string_view reason)
{
  if (!pending_trajectory_goal) return;
  auto result = std::make_shared<TrajectoryAction::Result>();
  result->error_code   = TrajectoryAction::Result::ERROR_EXECUTION_FAILED;
  result->error_string = std::string(reason);
  pending_trajectory_goal->abort(result);
  pending_trajectory_goal = nullptr;
  RCLCPP_INFO(logger, "Pending trajectory aborted: %.*s",
    static_cast<int>(reason.size()), reason.data());
}

bool ControllerManager::submit_joints(
  const std::vector<JointCommandTarget>& targets)
{
  if (!set_joint_controller.submit(targets, joint_state_manager.get_joint_states())) {
    RCLCPP_WARN(logger, "Rejected set_joints: controller busy");
    return false;
  }
  return true;
}

bool ControllerManager::submit_torques(
  const std::vector<Joint::JointIndex>& joints, bool enable)
{
  if (!torque_controller.submit(joints, enable)) {
    RCLCPP_WARN(logger, "Rejected set_torques: controller busy");
    return false;
  }
  return true;
}

bool ControllerManager::submit_mode_transition(
  const TransitionCommand& command, float& out_delay)
{
  bool success = false;
  switch (command.transition) {
  case TransitionCommand::TRANSITION_MODE_SWITCH:
    success = mode_transition_controller.submit_mode_switch(
      command.target_mode,
      joint_state_manager.get_joint_states(),
      out_delay);
    break;
  case TransitionCommand::TRANSITION_UPPER_BODY_CONTROL:
    success = mode_transition_controller.submit_upper_body_control(
      command.upper_body_enable,
      joint_state_manager.get_joint_states(),
      out_delay);
    break;
  default:
    break;
  }

  if (success)
    deactivate_lower_priority_controllers(ActiveController::mode_transition);

  return success;
}

bool ControllerManager::trajectory_busy() const
{
  return trajectory_controller.has_work() || pending_trajectory_goal != nullptr;
}

ControllerManager::ActiveController ControllerManager::select_active_controller() const
{
  if (mode_transition_controller.has_work()) return ActiveController::mode_transition;
  if (trajectory_controller.has_work() || pending_trajectory_goal) return ActiveController::trajectory;
  if (torque_controller.has_work())    return ActiveController::torque;
  if (set_joint_controller.has_work()) return ActiveController::set_joint;
  return ActiveController::none;
}

IController* ControllerManager::controller_for(ActiveController controller)
{
  switch (controller) {
  case ActiveController::mode_transition: return &mode_transition_controller;
  case ActiveController::trajectory:      return &trajectory_controller;
  case ActiveController::torque:          return &torque_controller;
  case ActiveController::set_joint:       return &set_joint_controller;
  case ActiveController::none:            return nullptr;
  }
  return nullptr;
}

bool ControllerManager::start_pending_trajectory_goal()
{
  if (!pending_trajectory_goal) return true;
  deactivate_lower_priority_controllers(ActiveController::trajectory);
  auto handle = pending_trajectory_goal;
  pending_trajectory_goal = nullptr;
  if (trajectory_controller.submit(handle)) {
    RCLCPP_INFO(logger, "Trajectory started from queue");
    return true;
  }
  RCLCPP_WARN(logger, "Trajectory failed to start");
  return false;
}

void ControllerManager::deactivate_lower_priority_controllers(
  ActiveController controller)
{
  if (controller == ActiveController::mode_transition) {
    abort_pending_trajectory(std::string("Preempted by mode-transition controller"));
    trajectory_controller.deactivate();
    torque_controller.deactivate();
    set_joint_controller.deactivate();
    if (active_controller != ActiveController::mode_transition)
      active_controller = ActiveController::none;
  }
  else if (controller == ActiveController::trajectory) {
    torque_controller.deactivate();
    set_joint_controller.deactivate();
    if (active_controller == ActiveController::torque ||
        active_controller == ActiveController::set_joint)
      active_controller = ActiveController::none;
  }
  else if (controller == ActiveController::torque) {
    set_joint_controller.deactivate();
    if (active_controller == ActiveController::set_joint)
      active_controller = ActiveController::none;
  }
}

std::string_view ControllerManager::controller_name(ActiveController controller)
{
  switch (controller) {
  case ActiveController::mode_transition: return "mode-transition";
  case ActiveController::trajectory:      return "trajectory-action";
  case ActiveController::torque:          return "set-torque";
  case ActiveController::set_joint:       return "set-joint";
  case ActiveController::none:            return "none";
  }
  return "unknown";
}

}  // namespace booster_controller