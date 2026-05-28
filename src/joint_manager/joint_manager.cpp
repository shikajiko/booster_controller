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
    const auto index = Joint::joint_to_index(target.joint);
    if (index >= Joint::kJointCnt) {
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
  q_reached = false;
}

void JointManager::handle_set_torques(const std::vector<Joint::JointIndex> & joints, bool torque_enable)
{
  if (joints.empty() || !has_joint_state() || command_running || should_publish_set_torque) {
    return;
  }

  std::vector<JointCommandTarget> target;

  for (auto joint : joints) {
    const auto index = Joint::joint_to_index(joint);
    if (index >= Joint::kJointCnt) {
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

bool JointManager::set_init_pose(float initial_weight, float target_weight)
{
  if (!has_joint_state() || command_running || should_publish_set_torque) {
    return false;
  }

  target_command.clear();
  active_command.clear();

  for (const auto joint : Joint::kAllJoints) {
    const auto index = Joint::joint_to_index(joint);
    const auto current_q = current_joint_states[index].q;
    const auto target_q = Joint::kStandPose[index];
    target_weight = std::clamp(target_weight, 0.0f, 1.0f);
    initial_weight = std::clamp(initial_weight, 0.0f, 1.0f);

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
  q_reached = false;
  return true;
}

bool JointManager::set_init_arms(float initial_weight, float target_weight) 
{
    if (!has_joint_state() || command_running || should_publish_set_torque) {
    return false;
  }

  target_command.clear();
  active_command.clear();

  for (const auto joint : Joint::kAllJoints) {
    const auto index = Joint::joint_to_index(joint);
    const auto current_q = current_joint_states[index].q;
    const auto target_q = Joint::is_arm_joint(joint)? Joint::kStandPose[index] : current_q;
    target_weight = std::clamp(target_weight, 0.0f, 1.0f);
    initial_weight = std::clamp(initial_weight, 0.0f, 1.0f);

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
  q_reached = false;
  return true;
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

  const bool need_increment_weight = target_command[0].weight - active_command[0].weight > 0.01f;
  const bool need_decrement_weight = active_command[0].weight - target_command[0].weight > 0.01f;

  if (need_increment_weight) {
    interpolate_weight(active_command[0].weight, target_command[0].weight);
    cmd = construct_joint_command(current_joint_states, active_command);
    return true;
  }

  if (need_decrement_weight && !q_reached) {
    q_reached = interpolate_q();
    cmd = construct_joint_command(current_joint_states, active_command);
    return true;
  }

  if (need_decrement_weight) {
    const bool done = interpolate_weight(active_command[0].weight, target_command[0].weight);
    cmd = construct_joint_command(current_joint_states, active_command);

    if (done) { 
      command_running = false; 
      q_reached = false; 
    }

    return true;
  }

  const bool done = interpolate_q();
  cmd = construct_joint_command(current_joint_states, active_command);

  if (done) command_running = false; 
  return true;
}

void JointManager::update_joint_state(const std::vector<MotorState> & state)
{
  current_joint_states = state;
  joint_state_received = true;
}

bool JointManager::interpolate_weight(float initial_weight, float target_weight) {
  bool weight_reached = true;
  bool increase_weight = target_weight > initial_weight;
  float weight_margin = increase_weight? Joint::kWeightMargin : -Joint::kWeightMargin;

  for (std::size_t i = 0; i < active_command.size(); i++) { 
    active_command[i].weight += weight_margin;
    active_command[i].weight = std::clamp(active_command[i].weight, 0.f, 1.f);

    if (std::abs(active_command[i].weight - target_command[i].weight) > 0.01F) {
        weight_reached = false;
    }
  }

  return weight_reached;
}

bool JointManager::interpolate_q() {
  bool command_reached = true;
  for (std::size_t i = 0; i < active_command.size(); i++) { 
    const auto velocity_scale = std::abs(active_command[i].velocity) > 0.0f ? std::abs(active_command[i].velocity) : 1.0f;
    const auto max_joint_delta = std::clamp(
      Joint::kBaseJointStep * velocity_scale, 0.0f, Joint::kMaxJointDelta);

    const auto delta_q = target_command[i].position - active_command[i].position;
    const auto joint_step = std::clamp(delta_q, -max_joint_delta, max_joint_delta);

    active_command[i].position += joint_step;

    if (std::abs(delta_q) > max_joint_delta) {
      command_reached = false;
    }
  }

  return command_reached;
}

bool JointManager::get_joint_state(
  Joint::JointIndex joint,
  booster_interface::msg::MotorState & state) const
{
  const auto index = Joint::joint_to_index(joint);
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
