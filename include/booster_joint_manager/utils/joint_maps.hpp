#pragma once

#include <array>
#include <cstddef>
#include <string_view>

#include "booster/robot/b1/b1_api_const.hpp"

namespace booster_joint_manager
{

namespace b1 = booster::robot::b1;

using JointIndex = b1::JointIndexK1;

inline constexpr std::size_t kJointCnt = b1::kJointCntK1;

inline constexpr std::array<JointIndex, kJointCnt> kAllJoints = {
  JointIndex::kHeadYaw,
  JointIndex::kHeadPitch,
  JointIndex::kLeftShoulderPitch,
  JointIndex::kLeftShoulderRoll,
  JointIndex::kLeftElbowPitch,
  JointIndex::kLeftElbowYaw,
  JointIndex::kRightShoulderPitch,
  JointIndex::kRightShoulderRoll,
  JointIndex::kRightElbowPitch,
  JointIndex::kRightElbowYaw,
  JointIndex::kLeftHipPitch,
  JointIndex::kLeftHipRoll,
  JointIndex::kLeftHipYaw,
  JointIndex::kLeftKneePitch,
  JointIndex::kCrankUpLeft,
  JointIndex::kCrankDownLeft,
  JointIndex::kRightHipPitch,
  JointIndex::kRightHipRoll,
  JointIndex::kRightHipYaw,
  JointIndex::kRightKneePitch,
  JointIndex::kCrankUpRight,
  JointIndex::kCrankDownRight,
};

inline constexpr std::array<float, kJointCnt> kInitJointPositions = {
  0, 0,
  0.0, -1.3, 0, -0.,
  0.0, 1.3, 0, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.,
  -0.0, 0, 0, 0.105, -0.10, 0.
};

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

inline constexpr std::array<JointIndex, 8> kArmJoints = {
  JointIndex::kLeftShoulderPitch,
  JointIndex::kLeftShoulderRoll,
  JointIndex::kLeftElbowPitch,
  JointIndex::kLeftElbowYaw,
  JointIndex::kRightShoulderPitch,
  JointIndex::kRightShoulderRoll,
  JointIndex::kRightElbowPitch,
  JointIndex::kRightElbowYaw,
};

std::string_view joint_name(JointIndex joint);
std::size_t joint_to_index(JointIndex joint);
bool is_arm_joint(JointIndex joint);

}  // namespace booster_joint_manager
