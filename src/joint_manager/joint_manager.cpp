#include "booster_joint_manager/joint_manager/joint_manager.hpp"

#include <algorithm>
#include <cmath>

namespace booster_joint_manager
{

void JointManager::handle_set_joints(const std::vector<JointCommandTarget> & targets)
{
  booster_interface::msg::LowState low_state;
  if (targets.empty() || !get_low_state(low_state)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (command_running) {
    return;
  }

  target_cmd = construct_joint_command(low_state, targets);
  active_cmd = target_cmd;

  for (std::size_t i = 0; i < active_cmd.motor_cmd.size() && i < low_state.motor_state_serial.size(); ++i) {
    active_cmd.motor_cmd[i].q = low_state.motor_state_serial[i].q;
  }

  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= active_cmd.motor_cmd.size()) {
      continue;
    }

    active_cmd.motor_cmd[index].weight = kInactiveJointWeight;
  }

  active_targets = targets;
  command_running = true;
}

bool JointManager::tick_command(booster_interface::msg::LowCmd & cmd)
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!command_running) {
    return false;
  }

  bool command_reached = true;

  for (const auto & target : active_targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= active_cmd.motor_cmd.size() || index >= target_cmd.motor_cmd.size()) {
      continue;
    }

    const auto target_q = target_cmd.motor_cmd[index].q;
    const auto current_q = active_cmd.motor_cmd[index].q;
    const auto target_weight = target_cmd.motor_cmd[index].weight;
    const auto current_weight = active_cmd.motor_cmd[index].weight;
    const auto velocity_scale = std::abs(target.velocity) > 0.0F ? std::abs(target.velocity) : 1.0F;
    const auto max_joint_delta = std::clamp(kBaseJointStep * velocity_scale, 0.0F, kMaxJointStep);

    const auto delta_q = target_q - current_q;
    const auto joint_step = std::clamp(delta_q, -max_joint_delta, max_joint_delta);
    const auto weight_step = std::clamp(target_weight - current_weight, -kWeightMargin, kWeightMargin);

    active_cmd.motor_cmd[index].q = current_q + joint_step;
    active_cmd.motor_cmd[index].weight = current_weight + weight_step;

    if (std::abs(delta_q) > max_joint_delta || std::abs(target_weight - current_weight) > kWeightMargin) {
          command_reached = false;
    }
  }

  cmd = active_cmd;
  if (command_reached) {
    command_running = false;
  }

  return true;
}

void JointManager::update_low_state(const booster_interface::msg::LowState & low_state)
{
  std::lock_guard<std::mutex> lock(mutex);
  current_low_state = low_state;
  low_state_received = true;
}

bool JointManager::get_joint_state(b1::JointIndex joint, booster_interface::msg::MotorState & joint_state) const 
{
  const auto index = joint_to_index(joint);
  std::lock_guard<std::mutex> lock(mutex);
  if (!low_state_received || index >= current_low_state.motor_state_serial.size()) {
    return false;
  }

  joint_state = current_low_state.motor_state_serial[index];
  return true;
}

bool JointManager::get_low_state(booster_interface::msg::LowState & low_state) const
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!low_state_received) {
    return false;
  }

  low_state = current_low_state;
  return true;
}

bool JointManager::has_low_state() const
{
  std::lock_guard<std::mutex> lock(mutex);
  return low_state_received;
}

}  // namespace booster_joint_manager
