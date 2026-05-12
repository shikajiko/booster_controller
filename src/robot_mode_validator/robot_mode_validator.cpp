#include "booster_joint_manager/robot_mode_validator/robot_mode_validator.hpp"

#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

bool RobotModeValidator::command_and_mode_match(
  const std::vector<JointCommandTarget> & targets,
  booster::robot::RobotMode current_mode,
  bool upper_control_enabled) const
{
  switch (current_mode) {
    case booster::robot::RobotMode::kCustom:
      return true;
    case booster::robot::RobotMode::kWalking:
    case booster::robot::RobotMode::kPrepare:
      if (!upper_control_enabled) {
        return false;
      }

      for (const auto & target : targets) {
        if (!is_arm_joint(target.joint)) {
          return false;
        }
      }

      return true;
    case booster::robot::RobotMode::kDamping:
    case booster::robot::RobotMode::kSoccer:
    case booster::robot::RobotMode::kUnknown:
    default:
      return false;
  }
}

}  // namespace booster_joint_manager
