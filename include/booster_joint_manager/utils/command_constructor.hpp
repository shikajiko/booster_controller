#pragma once

#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

inline constexpr float kDefaultJointDq = 0.0F;
inline constexpr float kDefaultJointTau = 0.0F;
inline constexpr float kDefaultJointWeight = 0.0F;
inline constexpr float kControlDt = 0.02F;
inline constexpr float kJointWeightRate = 0.2F;
inline constexpr float kBaseJointVelocity = 0.1F; //rad/s
inline constexpr float kMaxJointVelocity = 0.5F;  //rad/s
inline constexpr float kWeightMargin = kJointWeightRate * kControlDt;
inline constexpr float kBaseJointStep = kBaseJointVelocity * kControlDt;
inline constexpr float kMaxJointDelta = kMaxJointVelocity * kControlDt;
inline constexpr int kCommandFrequencyMs = static_cast<int>(kControlDt / 0.001F);

inline constexpr std::array<float, kJointCnt> kDefaultJointKps = {
  40., 40.,
  40., 50., 20., 20,
  40., 50., 20., 20,
  350., 350., 180., 350., 250., 250.,
  350., 350., 180., 350., 250., 250.
};

inline constexpr std::array<float, kJointCnt> kDefaultJointKds = {
  1.5, 1.5,
  0.5, 1.5, 0.2, 0.2,
  0.5, 1.5, 0.2, 0.2,
  7.5, 7.5, 3., 5.5, 5.0, 5.0,
  7.5, 7.5, 3., 5.5, 5.0, 5.0,
};

inline constexpr std::array<float, kJointCnt> kStandPose = {
    0, 0,
  0.0, -1.3, 0, -0.,
  0.0, 1.3, 0, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.
};

struct JointCommandTarget
{
  JointIndex joint;
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
