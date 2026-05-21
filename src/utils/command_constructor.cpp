#include "booster_joint_manager/utils/command_constructor.hpp"

namespace booster_joint_manager
{

booster_interface::msg::LowCmd construct_joint_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets)
{
  booster_interface::msg::LowCmd cmd;
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_PARALLEL;

  for (size_t i = 0; i< kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    motor_cmd.q = current_joint_state[i].q;
    motor_cmd.dq = kDefaultJointDq;
    motor_cmd.kp = kDefaultJointKps[i];
    motor_cmd.kd = kDefaultJointKds[i];
    motor_cmd.tau = kDefaultJointTau;
    motor_cmd.weight = kDefaultJointWeight;
    cmd.motor_cmd.push_back(motor_cmd);
  }

  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= kJointCnt) {
      continue;
    }
    cmd.motor_cmd.at(index).q       = target.position;
    cmd.motor_cmd.at(index).weight  = target.weight;
  }

  return cmd;
}

booster_interface::msg::LowCmd construct_set_torque_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets,
  bool enable_torque)
{
  booster_interface::msg::LowCmd cmd;
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_PARALLEL;

  for (size_t i = 0; i< kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    motor_cmd.q = current_joint_state[i].q;
    motor_cmd.dq = kDefaultJointDq;
    motor_cmd.kp = kDefaultJointKps[i];
    motor_cmd.kd = kDefaultJointKds[i];
    motor_cmd.tau = kDefaultJointTau;
    motor_cmd.weight = kDefaultJointWeight;
    cmd.motor_cmd.push_back(motor_cmd);
  }
  for (const auto target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= kJointCnt) continue;
    
    cmd.motor_cmd.at(index).q       = target.position;
    cmd.motor_cmd.at(index).dq      = kDefaultJointDq;
    cmd.motor_cmd.at(index).kp      = (enable_torque? kDefaultJointKps[index] : 0.);
    cmd.motor_cmd.at(index).kd      = (enable_torque? kDefaultJointKds[index] : 0.);
    cmd.motor_cmd.at(index).tau     = kDefaultJointTau;
    cmd.motor_cmd.at(index).weight  = target.weight;
  }

  return cmd;
}

}  // namespace booster_joint_manager
