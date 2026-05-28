#pragma once

#include <mutex>

#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/joint.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"

namespace booster_joint_manager
{

class JointManager
{
public:
  JointManager() = default;

  void update_joint_state(const std::vector<MotorState> & state);
  void handle_set_joints(const std::vector<JointCommandTarget> & targets);
  void handle_set_torques(const std::vector<Joint::JointIndex> & joints, bool torque_enable);
  bool tick_command(booster_interface::msg::LowCmd & cmd);

  bool get_joint_state(Joint::JointIndex joint, booster_interface::msg::MotorState & state) const;
  const std::vector<JointCommandTarget> & get_target_command() const;
  bool has_joint_state() const;
  bool set_init_pose(float initial_weight, float target_weight);
  bool set_init_arms(float initial_weight, float target_weight);
  bool interpolate_weight(float initial_weight, float target_weight);
  bool interpolate_q();

private:
  std::vector<MotorState> current_joint_states;
  bool joint_state_received{false};
  booster_interface::msg::LowCmd torque_command;
  std::vector<JointCommandTarget> target_command;
  std::vector<JointCommandTarget> active_command;
  bool should_publish_set_torque{false};
  bool command_running{false};
  bool q_reached{false};
};

}  // namespace booster_joint_manager
