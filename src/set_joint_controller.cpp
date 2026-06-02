#include "booster_controller/set_joint_controller.hpp"

#include <algorithm>
#include <cstddef>

#include "action_interface/msg/joint_trajectory.hpp"
#include "booster_controller/utils/command_constructor.hpp"

namespace booster_joint_manager
{

bool SetJointController::submit(
  const std::vector<JointCommandTarget>& targets,
  const std::vector<booster_interface::msg::MotorState>& current_states)
{
  if (targets.empty() || current_states.size() < Joint::kJointCnt || command_running) {
    return false;
  }

  auto target_positions = positions_from_state(current_states);
  for (const auto& target : targets) {
    const auto index = Joint::joint_to_index(target.joint);
    if (index >= Joint::kJointCnt) {
      continue;
    }
    target_positions[index] = std::clamp(
      static_cast<double>(target.position),
      static_cast<double>(Joint::kMinJointLimit[index]),
      static_cast<double>(Joint::kMaxJointLimit[index]));
  }

  action_interface::msg::JointTrajectory trajectory;
  action_interface::msg::JointTrajectoryPoint point;
  point.positions = target_positions;
  point.delay_before_seconds = 0.0;
  point.duration_seconds = 1.0;
  trajectory.points.push_back(point);

  interpolator.load(trajectory);
  interpolator.set_start_positions(positions_from_state(current_states));
  elapsed = 0.0;
  command_running = true;
  return true;
}

void SetJointController::deactivate()
{
  elapsed = 0.0;
  command_running = false;
}

void SetJointController::update(
  double dt,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  booster_interface::msg::LowCmd& command)
{
  if (!command_running || current_states.size() < Joint::kJointCnt) {
    return;
  }

  elapsed += dt;
  auto positions = interpolator.sample(elapsed);
  if (!positions) {
    positions = interpolator.end_position();
  }

  command = construct_joint_command(current_states, targets_from_positions(*positions));

  if (interpolator.is_done(elapsed)) {
    command_running = false;
    elapsed = 0.0;
  }
}

bool SetJointController::has_work() const
{
  return command_running;
}

std::vector<double> SetJointController::positions_from_state(
  const std::vector<booster_interface::msg::MotorState>& current_states)
{
  std::vector<double> positions;
  positions.reserve(Joint::kJointCnt);
  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    positions.push_back(current_states[i].q);
  }
  return positions;
}

std::vector<JointCommandTarget> SetJointController::targets_from_positions(
  const std::vector<double>& positions)
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(Joint::kJointCnt);
  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    targets.push_back({Joint::kAllJoints[i], static_cast<float>(positions[i]), 0.0F, 1.0F});
  }
  return targets;
}

}  // namespace booster_joint_manager
