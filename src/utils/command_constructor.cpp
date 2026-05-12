#include "booster_joint_manager/utils/command_constructor.hpp"

#include "keisan/angle/angle.hpp"

namespace booster_joint_manager
{

booster_interface::msg::LowCmd construct_joint_command(
  const booster_interface::msg::LowState & state,
  const std::vector<JointCommandTarget> & targets)
{
  booster_interface::msg::LowCmd cmd;
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;
  cmd.motor_cmd.resize(b1::kJointCnt);

  const auto & serial_states = state.motor_state_serial;

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);

    auto & joint_cmd = cmd.motor_cmd[index];
    joint_cmd.mode = 0x0A;
    joint_cmd.q = index < serial_states.size() ? serial_states[index].q : 0.0F;
    joint_cmd.dq = kDefaultJointDq;
    joint_cmd.tau = kDefaultJointTau;
    joint_cmd.kp = kDefaultJointKp;
    joint_cmd.kd = kDefaultJointKd;
    joint_cmd.weight = kInactiveJointWeight;
  }

  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= cmd.motor_cmd.size()) {
      continue;
    }

    auto & joint_cmd = cmd.motor_cmd[index];
    joint_cmd.q = keisan::make_degree(target.position).radian();
    joint_cmd.weight = kActiveJointWeight;
  }

  return cmd;
}

booster_interface::msg::LowCmd construct_set_torque_command(
  const booster_interface::msg::LowState & state,
  const std::vector<b1::JointIndex> & joints,
  bool enable_torque)
{
  booster_interface::msg::LowCmd cmd;
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;
  cmd.motor_cmd.resize(b1::kJointCnt);

  const auto & serial_states = state.motor_state_serial;
  
  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);

    auto & joint_cmd = cmd.motor_cmd[index];
    joint_cmd.mode = 0x0A;
    joint_cmd.q = index < serial_states.size() ? serial_states[index].q : 0.0F;
    joint_cmd.dq = kDefaultJointDq;
    joint_cmd.tau = kDefaultJointTau;
    joint_cmd.kp = enable_torque ? kDefaultJointKp : 0.0F;
    joint_cmd.kd = enable_torque ? kDefaultJointKd : 0.0F;
    joint_cmd.weight = kInactiveJointWeight;
  }

  for (const auto joint : joints) {
    const auto index = joint_to_index(joint);
    if (index >= cmd.motor_cmd.size()) {
      continue;
    }

    cmd.motor_cmd[index].weight = kActiveJointWeight;
  }

  return cmd;
}

}  // namespace booster_joint_manager
