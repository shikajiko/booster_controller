#pragma once

#include <vector>

#include "booster/robot/common/robot_shared.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"

namespace booster_joint_manager
{

class RobotModeValidator
{
public:
  RobotModeValidator() = default;

  bool command_and_mode_match(
    const std::vector<JointCommandTarget> & targets,
    booster::robot::RobotMode current_mode,
    bool upper_control_enabled) const;
};

}  // namespace booster_joint_manager
