#pragma once

#include <vector>

#include "booster_controller/utils/joint_command.hpp"
#include "booster_interface/msg/motor_state.hpp"

namespace booster_controller
{

using MotorState = booster_interface::msg::MotorState;

class JointStateManager {
public:
  void update_joint_state(const std::vector<MotorState>& state);
  bool get_joint_state(Joint::JointIndex joint, MotorState& state) const;
  const std::vector<MotorState>& get_joint_states() const;
  bool has_joint_state() const;

private:
  std::vector<MotorState> current_joint_states;
  bool joint_state_received = false;
};

}  // namespace booster_controller
