#include "booster_joint_manager/joint_manager/joint_manager.hpp"

#include <algorithm>
#include <cmath>

namespace booster_joint_manager
{

void JointManager::handle_set_joints(const std::vector<JointCommandTarget> & targets)
{

  if (!has_joint_state()) return;

  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= kJointCnt) {
      continue;
    }
    const auto current_q = current_joint_states[index].q;
    target_command.push_back(target);
    active_command.push_back(
      JointCommandTarget{
        target.joint,
        current_q,
        target.velocity,
        0.5,
      });
  }

  command_running = !active_command.empty();
}

void JointManager::handle_set_torques(const std::vector<JointIndex> & joints, bool torque_enable)
{
  if (joints.empty() || !has_joint_state() || command_running || should_publish_set_torque) {
    return;
  }

  std::vector<JointCommandTarget> target;

  for (auto joint : joints) {
    const auto index = joint_to_index(joint);
    if (index >= kJointCnt) {
      continue;
    }
    const auto current_q = current_joint_states[index].q;
    target.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.,
        0.5
      }
    );
  }

  torque_command = construct_set_torque_command(current_joint_states, target, torque_enable);
  should_publish_set_torque = true; 
}

bool JointManager::set_init_position(float target_weight, bool ignore_weight)
{
  if (!has_joint_state() || command_running || should_publish_set_torque) {
    return false;
  }

  target_command.clear();
  active_command.clear();

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);
    const auto current_q = current_joint_states[index].q;
    const auto target_q = kStandPose[index];
    float initial_weight = target_weight == 0.5? 0.0 : 0.5;
    if (ignore_weight) initial_weight = target_weight;

    target_command.push_back(
      JointCommandTarget{
        joint,
        target_q,
        1.,
        target_weight
      });

    active_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.,
        initial_weight,
      });
  }

  command_running = !active_command.empty();
  return true;
}

bool JointManager::interpolate_command(booster_interface::msg::LowCmd & cmd)
{
  if (should_publish_set_torque) {
    should_publish_set_torque = false;
    cmd = torque_command;
    return true;
  }

  if (!command_running) {
    return false;
  }

  bool command_reached = true;
  float weight_margin = active_command[0].weight - target_command[0].weight > 0? -kWeightMargin : kWeightMargin;

  // if current weight doesn't equal target weight, slowly move it toward desired weight
  if (std::abs(active_command[0].weight - target_command[0].weight) > 0.01) {
    for (std::size_t i = 0; i < active_command.size(); i++) { 
      active_command[i].weight += weight_margin;
      active_command[i].weight = std::clamp(active_command[i].weight, 0.f, 0.5f);

      if (std::abs(active_command[i].weight - target_command[i].weight) > 0.01F) {
        command_reached = false;
      }
    }
  }
  
  else {    
    for (std::size_t i = 0; i < active_command.size(); i++) { 
      const auto velocity_scale = std::abs(active_command[i].velocity) > 0.0F ? std::abs(active_command[i].velocity) : 1.0F;
      const auto max_joint_delta = std::clamp(kBaseJointStep * velocity_scale, 0.0F, kMaxJointDelta);

      const auto delta_q = target_command[i].position - active_command[i].position;
      const auto joint_step = std::clamp(delta_q, -max_joint_delta, max_joint_delta);

      active_command[i].position += joint_step;

      if (std::abs(delta_q) > max_joint_delta) {
        command_reached = false;
      }
    }
  }

  cmd = construct_joint_command(current_joint_states, active_command);
  if (command_reached) {
    command_running = false;
  }

  return true;
}

void JointManager::update_joint_state(const std::vector<MotorState> & state)
{
  current_joint_states = state;
  joint_state_received = true;
}

bool JointManager::get_joint_state(JointIndex joint, booster_interface::msg::MotorState & state) const
{
  const auto index = joint_to_index(joint);
  if (!has_joint_state() || index >= current_joint_states.size()) {
    return false;
  }

  state = current_joint_states[index];
  return true;
}

const std::vector<JointCommandTarget> & JointManager::get_target_command() const
{
  return target_command;
}

bool JointManager::has_joint_state() const
{
  return joint_state_received;
}

}  // namespace booster_joint_manager
