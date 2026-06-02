#include "booster_controller/joint_manager/joint_manager.hpp"

namespace booster_controller
{

void JointStateManager::update_joint_state(const std::vector<MotorState>& state)
{
  current_joint_states = state;
  joint_state_received = current_joint_states.size() >= Joint::kJointCnt;
}

bool JointStateManager::get_joint_state(Joint::JointIndex joint, MotorState& state) const
{
  const auto index = Joint::joint_to_index(joint);
  if (!has_joint_state() || index >= current_joint_states.size()) {
    return false;
  }

  state = current_joint_states[index];
  return true;
}

const std::vector<MotorState>& JointStateManager::get_joint_states() const
{
  return current_joint_states;
}

bool JointStateManager::has_joint_state() const
{
  return joint_state_received;
}

}  // namespace booster_controller
