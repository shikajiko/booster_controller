#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

std::string_view joint_name(b1::JointIndex joint)
{
  switch (joint) {
    case b1::JointIndex::kHeadYaw:
      return "HeadYaw";
    case b1::JointIndex::kHeadPitch:
      return "HeadPitch";
    case b1::JointIndex::kLeftShoulderPitch:
      return "LeftShoulderPitch";
    case b1::JointIndex::kLeftShoulderRoll:
      return "LeftShoulderRoll";
    case b1::JointIndex::kLeftElbowPitch:
      return "LeftElbowPitch";
    case b1::JointIndex::kLeftElbowYaw:
      return "LeftElbowYaw";
    case b1::JointIndex::kRightShoulderPitch:
      return "RightShoulderPitch";
    case b1::JointIndex::kRightShoulderRoll:
      return "RightShoulderRoll";
    case b1::JointIndex::kRightElbowPitch:
      return "RightElbowPitch";
    case b1::JointIndex::kRightElbowYaw:
      return "RightElbowYaw";
    case b1::JointIndex::kWaist:
      return "Waist";
    case b1::JointIndex::kLeftHipPitch:
      return "LeftHipPitch";
    case b1::JointIndex::kLeftHipRoll:
      return "LeftHipRoll";
    case b1::JointIndex::kLeftHipYaw:
      return "LeftHipYaw";
    case b1::JointIndex::kLeftKneePitch:
      return "LeftKneePitch";
    case b1::JointIndex::kCrankUpLeft:
      return "CrankUpLeft";
    case b1::JointIndex::kCrankDownLeft:
      return "CrankDownLeft";
    case b1::JointIndex::kRightHipPitch:
      return "RightHipPitch";
    case b1::JointIndex::kRightHipRoll:
      return "RightHipRoll";
    case b1::JointIndex::kRightHipYaw:
      return "RightHipYaw";
    case b1::JointIndex::kRightKneePitch:
      return "RightKneePitch";
    case b1::JointIndex::kCrankUpRight:
      return "CrankUpRight";
    case b1::JointIndex::kCrankDownRight:
      return "CrankDownRight";
  }

  return "Unknown";
}

std::size_t joint_to_index(b1::JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}

bool is_arm_joint(b1::JointIndex joint)
{
  for (const auto arm_joint : kArmJoints) {
    if (joint == arm_joint) {
      return true;
    }
  }

  return false;
}

}  // namespace booster_joint_manager
