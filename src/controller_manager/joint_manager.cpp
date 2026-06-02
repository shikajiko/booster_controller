#include "booster_controller/joint_manager/joint_manager.hpp"

namespace booster_joint_manager
{

void JointManager::update_joint_state(const std::vector<MotorState>& state)
{
  current_joint_states = state;
  joint_state_received = current_joint_states.size() >= Joint::kJointCnt;
}

bool JointManager::get_joint_state(Joint::JointIndex joint, MotorState& state) const
{
  const auto index = Joint::joint_to_index(joint);
  if (!has_joint_state() || index >= current_joint_states.size()) {
    return false;
  }

  state = current_joint_states[index];
  return true;
}

const std::vector<MotorState>& JointManager::get_joint_states() const
{
  return current_joint_states;
}

bool JointManager::has_joint_state() const
{
  return joint_state_received;
}

}  // namespace booster_joint_manager
