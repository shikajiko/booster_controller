#include "booster_controller/controller/mode_transition_controller.hpp"

#include <cstddef>

#include "action_interface/msg/joint_trajectory.hpp"
#include "booster_controller/utils/command_constructor.hpp"

namespace booster_controller
{

bool ModeTransitionController::submit_mode_switch(
  uint8_t target_mode,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  float& delay_second)
{
  if (command_running || current_states.size() < Joint::kJointCnt) {
    return false;
  }

  switch (target_mode) {
  case TransitionCommand::MODE_STAND:
    delay_second = 3.0F;
    return load_target_positions(
      std::vector<double>(Joint::kStandPose.begin(), Joint::kStandPose.end()),
      current_states,
      delay_second);
  case TransitionCommand::MODE_WALK:
    delay_second = 0.1F;
    return true;
  case TransitionCommand::MODE_CUSTOM:
    delay_second = 3.0F;
    return load_target_positions(
      std::vector<double>(Joint::kStandPose.begin(), Joint::kStandPose.end()),
      current_states,
      delay_second);
  default:
    delay_second = 0.0F;
    return false;
  }
}

bool ModeTransitionController::submit_upper_body_control(
  bool,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  float& delay_second)
{
  if (command_running || current_states.size() < Joint::kJointCnt) {
    return false;
  }

  delay_second = 3.0F;
  return load_target_positions(std::vector<double>(Joint::kStandPose.begin(), Joint::kStandPose.end()),
      current_states,
      delay_second);
}

void ModeTransitionController::deactivate()
{
  elapsed = 0.0;
  command_running = false;
}

void ModeTransitionController::update(
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
    deactivate();
  }
}

bool ModeTransitionController::has_work() const
{
  return command_running;
}

bool ModeTransitionController::load_target_positions(
  const std::vector<double>& target_positions,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  double duration_seconds)
{
  action_interface::msg::JointTrajectory trajectory;
  action_interface::msg::JointTrajectoryPoint point;
  point.positions = target_positions;
  point.delay_before_seconds = 0.0;
  point.duration_seconds = duration_seconds;
  trajectory.points.push_back(point);

  interpolator.load(trajectory);
  interpolator.set_start_positions(positions_from_state(current_states));
  elapsed = 0.0;
  command_running = true;
  return true;
}

std::vector<double> ModeTransitionController::positions_from_state(
  const std::vector<booster_interface::msg::MotorState>& current_states)
{
  std::vector<double> positions;
  positions.reserve(Joint::kJointCnt);
  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    positions.push_back(current_states[i].q);
  }
  return positions;
}

std::vector<JointCommandTarget> ModeTransitionController::targets_from_positions(
  const std::vector<double>& positions)
{
  std::vector<JointCommandTarget> targets;
  targets.reserve(Joint::kJointCnt);
  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    targets.push_back({Joint::kAllJoints[i], static_cast<float>(positions[i]), 0.0F, 1.0F});
  }
  return targets;
}

}  // namespace booster_controller
