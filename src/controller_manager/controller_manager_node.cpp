#include "booster_controller/node/controller_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

namespace booster_controller
{

ControllerManagerNode::ControllerManagerNode(const rclcpp::Node::SharedPtr& node) : node(node)
{
  joint_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
      update_joint_state(msg->motor_state_parallel);
    });

  joint_cmd_publisher =
    node->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 10);

  set_cmd_subscriber = node->create_subscription<booster_joint_interface::msg::SetJoints>(
    "joint/set_joints",
    10,
    [this](booster_joint_interface::msg::SetJoints::SharedPtr msg) {
      handle_set_joints(*msg);
    });

  set_torques_subscriber = node->create_subscription<booster_joint_interface::msg::SetTorques>(
    "joint/set_torques",
    10,
    [this](booster_joint_interface::msg::SetTorques::SharedPtr msg) {
      handle_set_torques(*msg);
    });

  prepare_transition_service = node->create_service<PrepareTransition>(
    "prep_transition_service",
    [this](
      const std::shared_ptr<PrepareTransition::Request> request,
      std::shared_ptr<PrepareTransition::Response> response) {
      handle_prepare_transition(request, response);
    });

  trajectory_action_server = rclcpp_action::create_server<TrajectoryAction>(
    node,
    "controller/run_trajectory",
    std::bind(
      &ControllerManagerNode::handle_trajectory_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&ControllerManagerNode::handle_trajectory_cancel, this, std::placeholders::_1),
    std::bind(&ControllerManagerNode::handle_trajectory_accepted, this, std::placeholders::_1));

  command_timer = node->create_wall_timer(
    std::chrono::milliseconds(Joint::kCommandFrequencyMs),
    [this]() {
      tick_controller();
    });

  RCLCPP_INFO(node->get_logger(), "Controller manager ready");
  RCLCPP_INFO(node->get_logger(), "Subscribed to /low_state");
  RCLCPP_INFO(node->get_logger(), "Publishing joint commands on /joint_ctrl");
  RCLCPP_INFO(node->get_logger(), "Subscribed to joint/set_joints");
  RCLCPP_INFO(node->get_logger(), "Subscribed to joint/set_torques");
  RCLCPP_INFO(node->get_logger(), "Prepare transition service ready on prep_transition_service");
  RCLCPP_INFO(node->get_logger(), "Trajectory action server ready on controller/run_trajectory");
}

void ControllerManagerNode::handle_set_joints(const booster_joint_interface::msg::SetJoints& msg)
{
  RCLCPP_INFO(
    node->get_logger(),
    "Set-joint command received: %zu joint targets",
    msg.joints.size());

  if (!joint_manager.has_joint_state()) {
    RCLCPP_WARN(node->get_logger(), "Rejected joint/set_joints: no joint state received");
    return;
  }

  const auto targets = joint_msg_to_target(msg);
  if (!set_joint_controller.submit(targets, joint_manager.get_joint_states())) {
    RCLCPP_WARN(node->get_logger(), "Rejected joint/set_joints: set-joint controller is busy");
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Set-joint controller queued successfully: %zu valid targets",
    targets.size());
}

void ControllerManagerNode::handle_set_torques(const booster_joint_interface::msg::SetTorques& msg)
{
  RCLCPP_INFO(
    node->get_logger(),
    "Set-torque command received: %zu joints, torque %s",
    msg.ids.size(),
    msg.torque_enable ? "enabled" : "disabled");

  const auto joints = id_to_joint_index(msg.ids);
  if (!torque_controller.submit(joints, msg.torque_enable)) {
    RCLCPP_WARN(node->get_logger(), "Rejected joint/set_torques: torque controller is busy");
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Set-torque controller queued successfully: %zu valid joints",
    joints.size());
}

void ControllerManagerNode::handle_prepare_transition(
  const std::shared_ptr<PrepareTransition::Request> request,
  std::shared_ptr<PrepareTransition::Response> response)
{
  RCLCPP_INFO(
    node->get_logger(),
    "Mode-transition command received: transition=%u target_mode=%u upper_body=%s",
    static_cast<unsigned int>(request->command.transition),
    static_cast<unsigned int>(request->command.target_mode),
    request->command.upper_body_enable ? "enabled" : "disabled");

  if (!joint_manager.has_joint_state()) {
    response->success = false;
    response->delay_second = 0.0F;
    RCLCPP_WARN(node->get_logger(), "Rejected transition: no joint state received");
    return;
  }

  const auto& command = request->command;
  float delay_second = 0.0F;
  bool success = false;
  switch (command.transition) {
  case TransitionCommand::TRANSITION_MODE_SWITCH:
    success = mode_transition_controller.submit_mode_switch(
      command.target_mode,
      joint_manager.get_joint_states(),
      delay_second);
    break;
  case TransitionCommand::TRANSITION_UPPER_BODY_CONTROL:
    success = mode_transition_controller.submit_upper_body_control(
      command.upper_body_enable,
      joint_manager.get_joint_states(),
      delay_second);
    break;
  default:
    success = false;
    delay_second = 0.0F;
    break;
  }

  response->success = success;
  response->delay_second = delay_second;

  if (success) {
    deactivate_lower_priority_controllers(ActiveController::mode_transition);
    RCLCPP_INFO(
      node->get_logger(),
      "Mode-transition controller started successfully: delay %.3f seconds",
      delay_second);
  } else {
    RCLCPP_WARN(node->get_logger(), "Rejected transition: invalid or busy transition command");
  }
}

rclcpp_action::GoalResponse ControllerManagerNode::handle_trajectory_goal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const TrajectoryAction::Goal> goal)
{
  RCLCPP_INFO(
    node->get_logger(),
    "Trajectory action goal received: %zu points",
    goal->trajectory.points.size());

  if (trajectory_controller.has_work() || pending_trajectory_goal) {
    RCLCPP_WARN(node->get_logger(), "Rejected trajectory goal: trajectory is active or queued");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(node->get_logger(), "Rejected trajectory goal: trajectory is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerManagerNode::handle_trajectory_cancel(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  try {
    RCLCPP_INFO(node->get_logger(), "Trajectory cancel received");
    if (pending_trajectory_goal && pending_trajectory_goal == goal_handle) {
      auto result = std::make_shared<TrajectoryAction::Result>();
      result->error_code = TrajectoryAction::Result::ERROR_OK;
      result->error_string = "Cancelled before start";
      pending_trajectory_goal->canceled(result);
      pending_trajectory_goal = nullptr;
      RCLCPP_INFO(node->get_logger(), "Queued trajectory action goal canceled");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    return trajectory_controller.cancel(goal_handle)
      ? rclcpp_action::CancelResponse::ACCEPT
      : rclcpp_action::CancelResponse::REJECT;
  } catch (const std::exception& error) {
    RCLCPP_WARN(
      node->get_logger(),
      "Rejected trajectory cancel: failed to load stand trajectory: %s",
      error.what());
    return rclcpp_action::CancelResponse::REJECT;
  }
}

void ControllerManagerNode::handle_trajectory_accepted(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  pending_trajectory_goal = goal_handle;
  RCLCPP_INFO(node->get_logger(), "Trajectory action goal queued");
}

void ControllerManagerNode::tick_controller()
{
  if (!joint_manager.has_joint_state()) {
    return;
  }

  const auto next_controller = select_active_controller();
  if (next_controller == ActiveController::none) {
    if (active_controller != ActiveController::none) {
      RCLCPP_INFO(
        node->get_logger(),
        "%.*s controller finished",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data());
      active_controller = ActiveController::none;
    }
    return;
  }

  if (next_controller != active_controller) {
    auto* previous_controller = controller_for(active_controller);
    if (previous_controller && previous_controller->has_work()) {
      previous_controller->deactivate();
      RCLCPP_INFO(
        node->get_logger(),
        "%.*s controller deactivated by higher-priority %.*s controller",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data(),
        static_cast<int>(controller_name(next_controller).size()),
        controller_name(next_controller).data());
    }

    active_controller = next_controller;
    auto* activated_controller = controller_for(active_controller);
    if (activated_controller) {
      activated_controller->activate();
      RCLCPP_INFO(
        node->get_logger(),
        "%.*s controller activated",
        static_cast<int>(controller_name(active_controller).size()),
        controller_name(active_controller).data());
    }
  }

  if (active_controller == ActiveController::trajectory && !trajectory_controller.has_work()) {
    if (!start_pending_trajectory_goal()) {
      active_controller = ActiveController::none;
      return;
    }
  }

  auto* controller = controller_for(active_controller);
  if (!controller) {
    return;
  }

  booster_interface::msg::LowCmd cmd;
  controller->update(Joint::kControlDt, joint_manager.get_joint_states(), cmd);
  RCLCPP_INFO_THROTTLE(
    node->get_logger(),
    *node->get_clock(),
    1000,
    "Publishing %.*s command",
    static_cast<int>(controller_name(active_controller).size()),
    controller_name(active_controller).data());
  RCLCPP_DEBUG(
    node->get_logger(),
    "Publishing %.*s command with %zu motor commands",
    static_cast<int>(controller_name(active_controller).size()),
    controller_name(active_controller).data(),
    cmd.motor_cmd.size());
  publish_joint_cmd(cmd);

  if (!controller->has_work()) {
    controller->deactivate();
    RCLCPP_INFO(
      node->get_logger(),
      "%.*s controller completed",
      static_cast<int>(controller_name(active_controller).size()),
      controller_name(active_controller).data());
    active_controller = ActiveController::none;
  }
}

ControllerManagerNode::ActiveController ControllerManagerNode::select_active_controller() const
{
  if (mode_transition_controller.has_work()) {
    return ActiveController::mode_transition;
  }
  if (trajectory_controller.has_work() || pending_trajectory_goal) {
    return ActiveController::trajectory;
  }
  if (torque_controller.has_work()) {
    return ActiveController::torque;
  }
  if (set_joint_controller.has_work()) {
    return ActiveController::set_joint;
  }
  return ActiveController::none;
}

IController* ControllerManagerNode::controller_for(ActiveController controller)
{
  switch (controller) {
  case ActiveController::mode_transition:
    return &mode_transition_controller;
  case ActiveController::trajectory:
    return &trajectory_controller;
  case ActiveController::torque:
    return &torque_controller;
  case ActiveController::set_joint:
    return &set_joint_controller;
  case ActiveController::none:
    return nullptr;
  }
  return nullptr;
}

bool ControllerManagerNode::start_pending_trajectory_goal()
{
  if (!pending_trajectory_goal) {
    return true;
  }

  deactivate_lower_priority_controllers(ActiveController::trajectory);

  auto goal_handle = pending_trajectory_goal;
  pending_trajectory_goal = nullptr;

  RCLCPP_INFO(node->get_logger(), "Starting queued trajectory action goal");
  if (trajectory_controller.submit(goal_handle)) {
    RCLCPP_INFO(node->get_logger(), "Trajectory action controller started successfully");
    return true;
  }

  RCLCPP_WARN(node->get_logger(), "Trajectory action controller failed to start");
  return false;
}

void ControllerManagerNode::abort_pending_trajectory_goal(std::string_view reason)
{
  if (!pending_trajectory_goal) {
    return;
  }

  auto result = std::make_shared<TrajectoryAction::Result>();
  result->error_code = TrajectoryAction::Result::ERROR_EXECUTION_FAILED;
  result->error_string = std::string(reason);
  pending_trajectory_goal->abort(result);
  pending_trajectory_goal = nullptr;
  RCLCPP_INFO(
    node->get_logger(),
    "Pending trajectory action goal aborted: %.*s",
    static_cast<int>(reason.size()),
    reason.data());
}

void ControllerManagerNode::deactivate_lower_priority_controllers(ActiveController controller)
{
  if (controller == ActiveController::mode_transition) {
    const bool had_trajectory = trajectory_controller.has_work();
    const bool had_torque = torque_controller.has_work();
    const bool had_set_joint = set_joint_controller.has_work();

    abort_pending_trajectory_goal("Preempted by mode-transition controller");
    trajectory_controller.deactivate();
    torque_controller.deactivate();
    set_joint_controller.deactivate();

    if (had_trajectory) {
      RCLCPP_INFO(
        node->get_logger(),
        "Trajectory action controller stopped for mode-transition controller");
    }
    if (had_torque) {
      RCLCPP_INFO(
        node->get_logger(),
        "Set-torque controller stopped for mode-transition controller");
    }
    if (had_set_joint) {
      RCLCPP_INFO(
        node->get_logger(),
        "Set-joint controller stopped for mode-transition controller");
    }
    if (active_controller != ActiveController::mode_transition) {
      active_controller = ActiveController::none;
    }
  } else if (controller == ActiveController::trajectory) {
    const bool had_torque = torque_controller.has_work();
    const bool had_set_joint = set_joint_controller.has_work();

    torque_controller.deactivate();
    set_joint_controller.deactivate();

    if (had_torque) {
      RCLCPP_INFO(
        node->get_logger(),
        "Set-torque controller stopped for trajectory action controller");
    }
    if (had_set_joint) {
      RCLCPP_INFO(
        node->get_logger(),
        "Set-joint controller stopped for trajectory action controller");
    }
    if (
      active_controller == ActiveController::torque ||
      active_controller == ActiveController::set_joint)
    {
      active_controller = ActiveController::none;
    }
  } else if (controller == ActiveController::torque) {
    const bool had_set_joint = set_joint_controller.has_work();

    set_joint_controller.deactivate();

    if (had_set_joint) {
      RCLCPP_INFO(
        node->get_logger(),
        "Set-joint controller stopped for set-torque controller");
    }
    if (active_controller == ActiveController::set_joint) {
      active_controller = ActiveController::none;
    }
  }
}

std::string_view ControllerManagerNode::controller_name(ActiveController controller)
{
  switch (controller) {
  case ActiveController::mode_transition:
    return "mode-transition";
  case ActiveController::trajectory:
    return "trajectory-action";
  case ActiveController::torque:
    return "set-torque";
  case ActiveController::set_joint:
    return "set-joint";
  case ActiveController::none:
    return "none";
  }
  return "unknown";
}

void ControllerManagerNode::update_joint_state(
  const std::vector<booster_interface::msg::MotorState>& msg)
{
  joint_manager.update_joint_state(msg);
}

bool ControllerManagerNode::get_joint_state(
  Joint::JointIndex joint,
  booster_interface::msg::MotorState& state) const
{
  return joint_manager.get_joint_state(joint, state);
}

void ControllerManagerNode::publish_joint_cmd(const booster_interface::msg::LowCmd& cmd)
{
  joint_cmd_publisher->publish(cmd);
}

std::vector<JointCommandTarget> ControllerManagerNode::joint_msg_to_target(
  const booster_joint_interface::msg::SetJoints& msg) const
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(msg.joints.size());

  for (const auto& joint : msg.joints) {
    if (joint.id >= Joint::kJointCnt) {
      continue;
    }
    const auto target_pos = std::clamp(
      joint.position,
      Joint::kMinJointLimit[joint.id],
      Joint::kMaxJointLimit[joint.id]);

    targets.push_back(
      JointCommandTarget{
        static_cast<Joint::JointIndex>(joint.id),
        target_pos,
        joint.velocity,
        1.0F});
  }

  return targets;
}

std::vector<Joint::JointIndex> ControllerManagerNode::id_to_joint_index(
  const std::vector<uint8_t>& ids) const
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

}  // namespace booster_controller
