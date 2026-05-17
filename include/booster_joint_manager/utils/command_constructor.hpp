#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

inline constexpr float kDefaultJointDq = 0.0F;
inline constexpr float kDefaultJointTau = 0.0F;
inline constexpr float kDefaultJointKp = 60.0F;
inline constexpr float kDefaultJointKd = 1.5F;
inline constexpr float kInactiveJointWeight = 0.0F;
inline constexpr float kActiveJointWeight = 0.5F;
inline constexpr float kControlDt = 0.02F;
inline constexpr float kJointWeightRate = 0.2F;
inline constexpr float kBaseJointVelocity = 0.1F;
inline constexpr float kMaxJointVelocity = 1.0F;
inline constexpr float kWeightMargin = kJointWeightRate * kControlDt;
inline constexpr float kBaseJointStep = kBaseJointVelocity * kControlDt;
inline constexpr float kBaseJointStepPerTick = kBaseJointStep;
inline constexpr float kMaxJointStep = kMaxJointVelocity * kControlDt;
inline constexpr float kMaxJointDelta = kMaxJointStep;
inline constexpr int kCommandTickMs = static_cast<int>(kControlDt / 0.001F);

struct JointCommandTarget
{
  JointIndex joint;
  float position;
  float velocity;
  float weight;
};

booster_interface::msg::LowCmd construct_joint_command(
  const std::vector<JointCommandTarget> & targets);

booster_interface::msg::LowCmd construct_set_torque_command(
  const std::vector<JointCommandTarget> & targets,
  bool enable_torque);

}  // namespace booster_joint_manager
