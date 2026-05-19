#pragma once

#include <mutex>

#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

class JointManager
{
public:
  JointManager() = default;

  void update_low_state(const booster_interface::msg::LowState & state);
  void handle_set_joints(const std::vector<JointCommandTarget> & targets);
  void handle_set_torques(const std::vector<JointIndex> & joints, bool torque_enable);
  bool tick_command(booster_interface::msg::LowCmd & cmd);
  bool get_joint_state(JointIndex joint, booster_interface::msg::MotorState & state) const;
  bool get_low_state(booster_interface::msg::LowState & state) const;
  const std::vector<JointCommandTarget> & get_target_command() const;
  bool has_low_state() const;
  void set_init_position(bool arm_only);
  void maintain_current_pose();

private:
  booster_interface::msg::LowState current_low_state;
  bool low_state_received{false};
  booster_interface::msg::LowCmd torque_command;
  std::vector<JointCommandTarget> target_command;
  std::vector<JointCommandTarget> active_command;
  bool should_publish_set_torque{false};
  bool command_running{false};
};

}  // namespace booster_joint_manager
