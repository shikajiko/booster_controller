#pragma once

#include "booster_model/joint_defaults.hpp"
#include "booster_model/joint_limits.hpp"
#include "booster_model/joint_map.hpp"
#include "booster_model/joint_poses.hpp"

namespace booster_controller
{

namespace Joint = booster_model::Joint;

using JointIndex = Joint::JointIndex;

struct JointCommandTarget
{
  Joint::JointIndex joint;
  float position;
  float velocity = 0.0F;
  float weight = 1.0F;
};

}  // namespace booster_controller
