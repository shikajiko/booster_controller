#include "booster_controller/controller/trajectory_controller.hpp"

#include <stdexcept>
#include <utility>

#include "booster_controller/utils/command_constructor.hpp"

namespace booster_controller
{

using Action = TrajectoryController::Action;

void TrajectoryController::activate()
{
  active = true;
}

void TrajectoryController::deactivate()
{
  if (active_goal) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::ERROR_EXECUTION_FAILED;
    result->error_string = "Trajectory controller deactivated";
    active_goal->abort(result);
    active_goal = nullptr;
  }

  active = false;
  elapsed = 0.0;
  returning_to_stand = false;
}

void TrajectoryController::update(
  double dt,
  const std::vector<booster_interface::msg::MotorState>& current_states,
  booster_interface::msg::LowCmd& command)
{
  if (!has_work() || current_states.size() < Joint::kJointCnt) {
    return;
  }

  if (elapsed == 0.0) {
    interpolator.set_start_positions(positions_from_state(current_states));
  }

  elapsed += dt;
  auto positions = interpolator.sample(elapsed);
  if (!positions) {
    positions = interpolator.end_position();
  }

  command = construct_joint_command(current_states, targets_from_positions(*positions));

  if (interpolator.is_done(elapsed)) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = 0;
    if (returning_to_stand) {
      result->error_string = "Cancelled, returned to stand pose";
      active_goal->canceled(result);
    } else {
      result->error_string = "OK";
      active_goal->succeed(result);
    }
    active_goal = nullptr;
    elapsed = 0.0;
    returning_to_stand = false;
    return;
  }

  if (active_goal) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->has_run_for_seconds = elapsed;
    feedback->desired.positions = *positions;
    feedback->actual.positions = positions_from_state(current_states);
    active_goal->publish_feedback(feedback);
  }
}

bool TrajectoryController::has_work() const
{
  return active_goal != nullptr;
}

bool TrajectoryController::is_idle() const
{
  return !has_work();
}

bool TrajectoryController::submit(std::shared_ptr<GoalHandle> goal_handle)
{
  if (!goal_handle || active_goal) {
    return false;
  }

  try {
    interpolator.load(goal_handle->get_goal()->trajectory);
  } catch (const std::exception& error) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::ERROR_INVALID_GOAL;
    result->error_string = error.what();
    goal_handle->abort(result);
    return false;
  }

  active_goal = std::move(goal_handle);
  elapsed = 0.0;
  returning_to_stand = false;
  return true;
}

bool TrajectoryController::cancel(std::shared_ptr<GoalHandle> goal_handle)
{
  if (!goal_handle || goal_handle != active_goal) {
    return false;
  }

  load_stand_trajectory();
  elapsed = 0.0;
  returning_to_stand = true;
  return true;
}

void TrajectoryController::load_stand_trajectory()
{
  action_interface::msg::JointTrajectory trajectory;
  action_interface::msg::JointTrajectoryPoint point;
  point.positions.assign(Joint::kStandPose.begin(), Joint::kStandPose.end());
  point.delay_before_seconds = 0.0;
  point.duration_seconds = 2.0;
  trajectory.points.push_back(point);
  interpolator.load(trajectory);
}

std::vector<double> TrajectoryController::positions_from_state(
  const std::vector<booster_interface::msg::MotorState>& current_states)
{
  std::vector<double> positions;
  positions.reserve(Joint::kJointCnt);
  for (std::size_t i = 0; i < Joint::kJointCnt; i++) {
    positions.push_back(current_states[i].q);
  }
  return positions;
}

std::vector<JointCommandTarget> TrajectoryController::targets_from_positions(
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
