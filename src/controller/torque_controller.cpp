#include "booster_controller/controller/torque_controller.hpp"

#include "booster_controller/utils/command_constructor.hpp"

namespace booster_controller
{

bool TorqueController::submit(const std::vector<Joint::JointIndex>& joints, bool enable_torque)
{
  if (joints.empty() || command_pending) {
    return false;
  }

  pending_joints = joints;
  pending_torque_enable = enable_torque;
  command_pending = true;
  return true;
}

void TorqueController::deactivate()
{
  pending_joints.clear();
  pending_torque_enable = false;
  command_pending = false;
}

void TorqueController::update(
  double,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  booster_interface::msg::LowCmd& command)
{
  if (!command_pending || current_states.size() < Joint::kJointCnt) {
    return;
  }

  std::vector<JointCommandTarget> targets;
  targets.reserve(pending_joints.size());
  for (const auto joint : pending_joints) {
    const auto index = Joint::joint_to_index(joint);
    if (index >= Joint::kJointCnt) {
      continue;
    }
    targets.push_back({joint, current_states[index].q, 0.0F, 1.0F});
  }

  command = construct_set_torque_command(current_states, targets, pending_torque_enable);
  deactivate();
}

bool TorqueController::has_work() const
{
  return command_pending;
}

}  // namespace booster_controller
