#include "booster_controller/utils/command_constructor.hpp"

namespace booster_controller
{

booster_interface::msg::LowCmd construct_joint_command(
  const std::vector<MotorState> & current_joint_state,
  const std::vector<JointCommandTarget> & targets)
{
  booster_interface::msg::LowCmd cmd;
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_PARALLEL;

  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    motor_cmd.q = current_joint_state[i].q;
    motor_cmd.dq = Joint::kDefaultJointDq;
    motor_cmd.kp = Joint::kDefaultJointKps[i];
    motor_cmd.kd = Joint::kDefaultJointKds[i];
    motor_cmd.tau = Joint::kDefaultJointTau;
    motor_cmd.weight = Joint::kDefaultJointWeight;
    cmd.motor_cmd.push_back(motor_cmd);
  }

  for (const auto & target : targets) {
    const auto index = Joint::joint_to_index(target.joint);
    if (index >= Joint::kJointCnt) {
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

  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    motor_cmd.q = current_joint_state[i].q;
    motor_cmd.dq = Joint::kDefaultJointDq;
    motor_cmd.kp = Joint::kDefaultJointKps[i];
    motor_cmd.kd = Joint::kDefaultJointKds[i];
    motor_cmd.tau = Joint::kDefaultJointTau;
    motor_cmd.weight = Joint::kDefaultJointWeight;
    cmd.motor_cmd.push_back(motor_cmd);
  }
  for (const auto target : targets) {
    const auto index = Joint::joint_to_index(target.joint);
    if (index >= Joint::kJointCnt) continue;

    cmd.motor_cmd.at(index).q       = target.position;
    cmd.motor_cmd.at(index).dq      = Joint::kDefaultJointDq;
    cmd.motor_cmd.at(index).kp      = (enable_torque? Joint::kDefaultJointKps[index] : 0.);
    cmd.motor_cmd.at(index).kd      = (enable_torque? Joint::kDefaultJointKds[index] : 0.);
    cmd.motor_cmd.at(index).tau     = Joint::kDefaultJointTau;
    cmd.motor_cmd.at(index).weight  = target.weight;
  }

  return cmd;
}

}  // namespace booster_controller
