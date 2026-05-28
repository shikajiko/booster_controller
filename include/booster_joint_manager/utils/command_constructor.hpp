#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/joint.hpp"

namespace booster_joint_manager
{

struct JointCommandTarget
{
  Joint::JointIndex joint;
  float position;
  float velocity;
  float weight;
};

using MotorState = booster_interface::msg::MotorState;

booster_interface::msg::LowCmd construct_joint_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets);

booster_interface::msg::LowCmd construct_set_torque_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets,
  bool enable_torque);

}  // namespace booster_joint_manager
