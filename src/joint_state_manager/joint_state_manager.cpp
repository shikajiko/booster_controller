#include "booster_controller/joint_state_manager/joint_state_manager.hpp"
#include <cmath>

namespace booster_controller
{

void JointStateManager::update_joint_state(const std::vector<MotorState>& state)
{
  current_joint_states = state;
  joint_state_received = current_joint_states.size() >= Joint::kJointCnt;

  if (current_joint_degrees.joints.size() != current_joint_states.size()) {
      current_joint_degrees.joints.resize(current_joint_states.size());

      for (size_t i = 0; i < current_joint_states.size(); ++i) {
          current_joint_degrees.joints[i].id = static_cast<uint8_t>(i);
      }
  }

  constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);

  for (size_t i = 0; i < current_joint_states.size(); ++i) {
    current_joint_degrees.joints[i].position =
        current_joint_states[i].q * kRadToDeg;
  }
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

const CurrentJoints& JointStateManager::get_joint_degrees() const
{
  return current_joint_degrees;
}

bool JointStateManager::has_joint_state() const
{
  return joint_state_received;
}

}  // namespace booster_controller
