#include "booster_controller/controller_manager/controller_manager_node.hpp"

#include <algorithm>

namespace booster_controller
{

ControllerManagerNode::ControllerManagerNode(rclcpp::Node::SharedPtr node)
  : node(node), joint_state_manager(), controller_manager(node, joint_state_manager)
{
  joint_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
      handle_joint_state(msg);
    });

  gripper_state_subscriber = node->create_subscription<booster_joint_interface::msg::SetJoints>(
    "/joint/gripper_state",
    10,
    [this](booster_joint_interface::msg::SetJoints::SharedPtr msg) {
      handle_gripper_state(msg);
    });

  current_joints_publisher = 
    node->create_publisher<booster_joint_interface::msg::SetJoints>("/joint/current_joints", 10);

  joint_command_publisher =
    node->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 10);

  gripper_torque_publisher = 
    node->create_publisher<booster_joint_interface::msg::SetTorques>("/joint/set_gripper_torques", 10);

  set_gripper_publisher = 
    node->create_publisher<booster_joint_interface::msg::SetJoints>("/joint/set_grippers", 10);

  set_joints_subscriber =
    node->create_subscription<booster_joint_interface::msg::SetJoints>(
      "/joint/set_joints",
      10,
      [this](booster_joint_interface::msg::SetJoints::SharedPtr msg) {
        handle_set_joints(msg);
      });

  set_torques_subscriber =
    node->create_subscription<booster_joint_interface::msg::SetTorques>(
      "/joint/set_torques",
      10,
      [this](booster_joint_interface::msg::SetTorques::SharedPtr msg) {
        handle_set_torques(msg);
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
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const TrajectoryAction::Goal> goal) {
      return handle_trajectory_goal(uuid, goal);
    },
    [this](std::shared_ptr<TrajectoryGoalHandle> goal_handle) {
      return handle_trajectory_cancel(goal_handle);
    },
    [this](std::shared_ptr<TrajectoryGoalHandle> goal_handle) {
      handle_trajectory_accepted(goal_handle);
    });

  timer = node->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(Joint::kCommandFrequencyMs)),
    [this]() { tick(); });

  RCLCPP_INFO(node->get_logger(), "Controller manager node ready");
}

void ControllerManagerNode::tick()
{
  current_joints_publisher->publish(joint_state_manager.get_joint_degrees());

  auto output = controller_manager.update(
    Joint::kControlDt,
    joint_state_manager.get_joint_states());

  if (output.low_cmd) {
    joint_command_publisher->publish(*output.low_cmd);
  }

  if (output.gripper_cmd) {
    set_gripper_publisher->publish(*output.gripper_cmd);
  }
}

void ControllerManagerNode::handle_joint_state(
  booster_interface::msg::LowState::SharedPtr msg)
{
  joint_state_manager.update_joint_state(msg->motor_state_parallel);
}

void ControllerManagerNode::handle_gripper_state(
  booster_joint_interface::msg::SetJoints::SharedPtr msg)
{
  joint_state_manager.update_gripper_state(*msg);
}

void ControllerManagerNode::handle_set_joints(
  booster_joint_interface::msg::SetJoints::SharedPtr msg)
{
  if (!controller_manager.submit_joints(msg_to_targets(*msg))) {
    RCLCPP_WARN(node->get_logger(), "set_joints rejected");
  }
}

void ControllerManagerNode::handle_set_torques(
  booster_joint_interface::msg::SetTorques::SharedPtr msg)
{
  std::vector<uint8_t> gripper_ids;

  for (int i = 0; i < msg->ids.size(); i++) {
    if (msg->ids[i] >= Joint::kJointCnt) 
    {
      gripper_ids.push_back(msg->ids[i]);
    }
  }

  auto joints = id_to_joint_idx(msg->ids);

  if (!controller_manager.submit_torques(joints, msg->torque_enable)) {
    RCLCPP_WARN(node->get_logger(), "set_torques rejected");
  }

  booster_joint_interface::msg::SetTorques gripper_msg;
  for (const auto& id : gripper_ids) {
    gripper_msg.ids.push_back(id);
    gripper_msg.torque_enable = msg->torque_enable;
    gripper_torque_publisher->publish(gripper_msg);
  }
}

void ControllerManagerNode::handle_prepare_transition(
  const std::shared_ptr<PrepareTransition::Request> request,
  std::shared_ptr<PrepareTransition::Response> response)
{
  float delay = 0.0F;

  response->success =
    controller_manager.submit_mode_transition(request->command, delay);
  response->delay_second = delay;

  if (!response->success) {
    RCLCPP_WARN(node->get_logger(), "Mode transition rejected");
  }
}

rclcpp_action::GoalResponse ControllerManagerNode::handle_trajectory_goal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const TrajectoryAction::Goal> goal)
{
  if (controller_manager.trajectory_busy()) {
    RCLCPP_WARN(node->get_logger(), "Trajectory goal rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(node->get_logger(), "Trajectory goal rejected: empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  for (const auto& point : goal->trajectory.points) {
    const auto size = point.positions.size();
    if (size != Joint::kJointCnt && size != Joint::kTotalJointCnt) {
      RCLCPP_WARN(node->get_logger(),
        "Trajectory goal rejected: point has %zu positions, expected %zu or %zu",
        size, Joint::kJointCnt, Joint::kTotalJointCnt);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerManagerNode::handle_trajectory_cancel(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  try {
    return controller_manager.cancel_trajectory(goal_handle)
      ? rclcpp_action::CancelResponse::ACCEPT
      : rclcpp_action::CancelResponse::REJECT;
  } catch (const std::exception& error) {
    RCLCPP_WARN(node->get_logger(), "Cancel failed: %s", error.what());
    return rclcpp_action::CancelResponse::REJECT;
  }
}

void ControllerManagerNode::handle_trajectory_accepted(
  std::shared_ptr<TrajectoryGoalHandle> goal_handle)
{
  controller_manager.submit_trajectory(goal_handle);
}

std::vector<JointCommandTarget> ControllerManagerNode::msg_to_targets(
  const booster_joint_interface::msg::SetJoints& msg) const
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(msg.joints.size());

  for (const auto& joint : msg.joints) {
    if (joint.id >= Joint::kJointCnt) {
      continue;
    }

    const auto position = std::clamp(
      joint.position,
      Joint::kMinJointLimit[joint.id],
      Joint::kMaxJointLimit[joint.id]);

    targets.push_back(
      {
        static_cast<Joint::JointIndex>(joint.id),
        position,
        joint.velocity,
        1.0F
      });
  }

  return targets;
}

std::vector<Joint::JointIndex> ControllerManagerNode::id_to_joint_idx(
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