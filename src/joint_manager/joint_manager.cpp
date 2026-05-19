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

  const auto & current_motor_states = current_low_state.motor_state_serial;
  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= kJointCnt) {
      continue;
    }

    const auto current_q = index < current_motor_states.size() ? current_motor_states[index].q : 0.0F;
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
  if (joints.empty() || !has_low_state() || command_running || should_publish_set_torque) {
    return;
  }

  std::vector<JointCommandTarget> target;
  const auto & current_motor_states = current_low_state.motor_state_serial;

  for (auto joint : joints) {
    int index = joint_to_index(joint);
    target.push_back(
      JointCommandTarget{
        joint,
        current_motor_states[index].q,
        1.,
        0.5
      }
    );
  }

  torque_command = construct_set_torque_command(target, torque_enable);
  should_publish_set_torque = true; 

}

void JointManager::set_init_position(bool arm_only)
{
  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  const auto & current_motor_states = current_low_state.motor_state_serial;

  for (const auto joint : kAllJoints) {
    if (!is_arm_joint(joint) && arm_only) continue;
    const auto index = joint_to_index(joint);
    const auto current_q = current_motor_states[index].q;
    const auto target_q = kInitJointPositions[index];

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

void JointManager::maintain_current_pose()
{
  if (command_running || should_publish_set_torque || !has_low_state()) {
    return;
  }
  target_command.clear();
  active_command.clear();

  const auto & current_motor_states = current_low_state.motor_state_serial;

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);
    const auto current_q = current_motor_states[index].q;

    target_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.,
        0.5,
      });

    active_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.,
        0.,
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
      const auto max_joint_delta = std::clamp(kBaseJointStep * velocity_scale, 0.0F, kMaxJointStep);

      const auto delta_q = target_command[i].position - active_command[i].position;
      const auto joint_step = std::clamp(delta_q, -max_joint_delta, max_joint_delta);

      active_command[i].position += joint_step;

      if (std::abs(delta_q) > max_joint_delta) {
        command_reached = false;
      }
    }
  }

  cmd = construct_joint_command(active_command);
  if (command_reached) {
    command_running = false;
  }

  return true;
}

void JointManager::update_low_state(const booster_interface::msg::LowState & low_state)
{
  current_low_state = low_state;
  low_state_received = true;
}

bool JointManager::get_joint_state(JointIndex joint, booster_interface::msg::MotorState & joint_state) const
{
  const auto index = joint_to_index(joint);
  if (!low_state_received || index >= current_low_state.motor_state_serial.size()) {
    return false;
  }

  joint_state = current_low_state.motor_state_serial[index];
  return true;
}

bool JointManager::get_low_state(booster_interface::msg::LowState & low_state) const
{
  if (!low_state_received) {
    return false;
  }

  low_state = current_low_state;
  return true;
}

const std::vector<JointCommandTarget> & JointManager::get_target_command() const
{
  return target_command;
}

bool JointManager::has_low_state() const
{
  return low_state_received;
}

}  // namespace booster_joint_manager
