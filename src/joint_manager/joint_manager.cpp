#include "booster_joint_manager/joint_manager/joint_manager.hpp"

#include <algorithm>
#include <cmath>

namespace booster_joint_manager
{

void JointManager::handle_set_joints(const std::vector<JointCommandTarget> & targets)
{

  if (!has_low_state()) return;

  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  const auto current_q = current_joint_states[index].q;
  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= kJointCnt) {
      continue;
    }
    target_command.push_back(target);
    active_command.push_back(
      JointCommandTarget{
        target.joint,
        current_q;
        target.velocity,
        0.5,
      });
  }

  command_running = !active_command.empty();
}

void JointManager::handle_set_torques(const std::vector<JointIndex> & joints, bool torque_enable)
{
  if (joints.empty() || !has_low_state() || command_running || should_publish_set_torque) {
    return;
  }

  std::vector<JointCommandTarget> target;
  const auto current_q = current_joint_states[index].q;

  for (auto joint : joints) {
    int index = joint_to_index(joint);
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

void JointManager::set_init_position()
{
  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);
    const auto current_q = current_joint_states[index].q;
    const auto target_q = kStandPose[index];

    target_command.push_back(
      JointCommandTarget{
        joint,
        target_q,
        1.,
        0.5,
      });

    active_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.,
        0,
      });
  }

  command_running = !active_command.empty();
}

bool JointManager::tick_command(booster_interface::msg::LowCmd & cmd)
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

  // if weight is < 0.5, slowly increment weight before moving joint
  if (active_command[0].weight < 0.5) {
    for (std::size_t i = 0; i < active_command.size(); i++) { 
      active_command[i].weight += kWeightMargin;
      active_command[i].weight = std::clamp(active_command[i].weight, 0.f, 0.5f);

      if (active_command[i].weight < 0.5) {
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
  current_joint_state = state;
  joint_state_received = true;
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
