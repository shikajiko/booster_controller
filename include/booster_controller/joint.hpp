#pragma once

#include <array>
#include <cstddef>
#include <string_view>

#include "booster/robot/b1/b1_api_const.hpp"

namespace booster_joint_manager
{

struct JointCommandTarget
{
  Joint::JointIndex joint;
  float position;
  float velocity = 0;
  float weight = 1;
};

}

namespace booster_joint_manager::Joint
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

inline constexpr float kDefaultJointDq = 0.0F;
inline constexpr float kDefaultJointTau = 0.0F;
inline constexpr float kDefaultJointWeight = 0.0F;
inline constexpr float kControlDt = 0.02F;
inline constexpr float kJointWeightRate = 0.2F;
inline constexpr float kBaseJointVelocity = 0.1F;  // rad/s
inline constexpr float kMaxJointVelocity = 0.5F;  // rad/s
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

inline constexpr std::array<float, kJointCnt> kMinJointLimit = {
  -1.029744F,  // Head Yaw Joint (-59 deg)
  -0.331613F,  // Head Pitch Joint (-19 deg)
  -2.949606F,  // Left Shoulder Pitch Joint (-169 deg)
  -1.640609F,  // Left Shoulder Roll Joint (-94 deg)
  -1.902409F,  // Left Shoulder Yaw Joint (-109 deg)
  -2.251475F,  // Left Elbow Joint (-129 deg)
  -2.949606F,  // Right Shoulder Pitch Joint (-169 deg)
  -1.640609F,  // Right Shoulder Roll Joint (-94 deg)
  -1.902409F,  // Right Shoulder Yaw Joint (-109 deg)
  -0.680678F,  // Right Elbow Joint (-39 deg)
  -2.967060F,  // Left Hip Pitch Joint (-170 deg)
  -0.383972F,  // Left Hip Roll Joint (-22 deg)
  -1.029744F,  // Left Hip Yaw Joint (-59 deg)
  0.000000F,  // Left Knee Joint (0 deg)
  -0.296706F,  // Left Ankle Up Joint (-17 deg)
  -0.279253F,  // Left Ankle Down Joint (-16 deg)
  -2.967060F,  // Right Hip Pitch Joint (-170 deg)
  -1.553343F,  // Right Hip Roll Joint (-89 deg)
  -1.029744F,  // Right Hip Yaw Joint (-59 deg)
  0.000000F,  // Right Knee Joint (0 deg)
  -0.296706F,  // Right Ankle Up Joint (-17 deg)
  -0.279253F  // Right Ankle Down Joint (-16 deg)
};

inline constexpr std::array<float, kJointCnt> kMaxJointLimit = {
  1.029744F,  // Head Yaw Joint (59 deg)
  0.855211F,  // Head Pitch Joint (49 deg)
  1.204277F,  // Left Shoulder Pitch Joint (69 deg)
  1.640609F,  // Left Shoulder Roll Joint (94 deg)
  1.902409F,  // Left Shoulder Yaw Joint (109 deg)
  0.680678F,  // Left Elbow Joint (39 deg)
  1.204277F,  // Right Shoulder Pitch Joint (69 deg)
  1.640609F,  // Right Shoulder Roll Joint (94 deg)
  1.902409F,  // Right Shoulder Yaw Joint (109 deg)
  2.251475F,  // Right Elbow Joint (129 deg)
  2.234021F,  // Left Hip Pitch Joint (128 deg)
  1.553343F,  // Left Hip Roll Joint (89 deg)
  1.029744F,  // Left Hip Yaw Joint (59 deg)
  2.321288F,  // Left Knee Joint (133 deg)
  0.663225F,  // Left Ankle Up Joint (38 deg)
  0.715585F,  // Left Ankle Down Joint (41 deg)
  2.234021F,  // Right Hip Pitch Joint (128 deg)
  0.383972F,  // Right Hip Roll Joint (22 deg)
  1.029744F,  // Right Hip Yaw Joint (59 deg)
  2.321288F,  // Right Knee Joint (133 deg)
  0.663225F,  // Right Ankle Up Joint (38 deg)
  0.715585F  // Right Ankle Down Joint (41 deg)
};

constexpr std::string_view joint_name(JointIndex joint)
{
  switch (joint) {
  case JointIndex::kHeadYaw:
    return "HeadYaw";
  case JointIndex::kHeadPitch:
    return "HeadPitch";
  case JointIndex::kLeftShoulderPitch:
    return "LeftShoulderPitch";
  case JointIndex::kLeftShoulderRoll:
    return "LeftShoulderRoll";
  case JointIndex::kLeftElbowPitch:
    return "LeftElbowPitch";
  case JointIndex::kLeftElbowYaw:
    return "LeftElbowYaw";
  case JointIndex::kRightShoulderPitch:
    return "RightShoulderPitch";
  case JointIndex::kRightShoulderRoll:
    return "RightShoulderRoll";
  case JointIndex::kRightElbowPitch:
    return "RightElbowPitch";
  case JointIndex::kRightElbowYaw:
    return "RightElbowYaw";
  case JointIndex::kLeftHipPitch:
    return "LeftHipPitch";
  case JointIndex::kLeftHipRoll:
    return "LeftHipRoll";
  case JointIndex::kLeftHipYaw:
    return "LeftHipYaw";
  case JointIndex::kLeftKneePitch:
    return "LeftKneePitch";
  case JointIndex::kCrankUpLeft:
    return "CrankUpLeft";
  case JointIndex::kCrankDownLeft:
    return "CrankDownLeft";
  case JointIndex::kRightHipPitch:
    return "RightHipPitch";
  case JointIndex::kRightHipRoll:
    return "RightHipRoll";
  case JointIndex::kRightHipYaw:
    return "RightHipYaw";
  case JointIndex::kRightKneePitch:
    return "RightKneePitch";
  case JointIndex::kCrankUpRight:
    return "CrankUpRight";
  case JointIndex::kCrankDownRight:
    return "CrankDownRight";
  }

  return "Unknown";
}

constexpr std::size_t joint_to_index(JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}

constexpr bool is_arm_joint(JointIndex joint)
{
  for (const auto arm_joint : kArmJoints) {
    if (joint == arm_joint) {
      return true;
    }
  }

  return false;
}

}  // namespace booster_joint_manager::Joint

namespace booster_joint_manager
{

using JointIndex = Joint::JointIndex;

}  // namespace booster_joint_manager
