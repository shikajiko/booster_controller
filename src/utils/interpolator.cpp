#include "booster_controller/utils/interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace booster_controller
{
namespace
{
// Looser tolerance used only to decide whether the *entire* trajectory has
// finished. Distinct from Joint::kMaxJointDelta, which gates advancing
// between intermediate steps.
constexpr double kFinalPositionTolerance = 0.1;
}  // namespace

void Interpolator::load(const booster_action_interface::msg::JointTrajectory& trajectory)
{
  steps.clear();
  final_position.clear();
  last_sent_position.clear();
  delta_steps.clear();
  total_duration = 0.0;
  current_step_index = 0;
  last_step_index = std::numeric_limits<std::size_t>::max();
  tick_count_for_current_step = 0;
  joint_count = 0;

  if (trajectory.points.empty()) {
    throw std::invalid_argument("Trajectory has no points");
  }

  const auto first_size = trajectory.points.front().positions.size();
  if (first_size != Joint::kJointCnt && first_size != Joint::kTotalJointCnt) {
    throw std::invalid_argument("Number of joints does not match a known joint count");
  }
  joint_count = first_size;

  double cursor = 0.0;
  for (std::size_t i = 0; i < trajectory.points.size(); i++) {
    const auto& point = trajectory.points[i];

    if (point.positions.size() != joint_count) {
      throw std::invalid_argument("All trajectory points must have the same joint count");
    }

    std::vector<double> start_position =
      (i == 0) ? std::vector<double>(joint_count, 0.0)
               : trajectory.points[i - 1].positions;

    cursor += point.delay_before_seconds;
    const double start_time = cursor;
    cursor += point.duration_seconds;
    const double end_time = cursor;

    steps.push_back({start_time, end_time, start_position, point.positions});
  }

  total_duration = cursor;
  final_position = trajectory.points.back().positions;
  last_sent_position = steps.front().start_position;
}

void Interpolator::set_start_positions(const std::vector<double>& current_joint_position)
{
  if (current_joint_position.size() != joint_count) {
    return;
  }

  if (!steps.empty()) {
    steps.front().start_position = current_joint_position;
    last_sent_position = current_joint_position;
  }
}

std::optional<std::vector<double>> Interpolator::sample(double time_seconds)
{
  if (steps.empty()) {
    return std::nullopt;
  }

  while (current_step_index + 1 < steps.size()) {
    const auto& step = steps[current_step_index];
    if (time_seconds >= step.end_time &&
        all_reached(last_sent_position, step.end_position, Joint::kMaxJointDelta)) {
      current_step_index++;
    } else {
      break;
    }
  }

  const auto& step = steps[current_step_index];
  if (time_seconds < step.start_time) {
    return last_sent_position;
  }

  const bool is_starting_new_step = (current_step_index != last_step_index);
  if (is_starting_new_step) {
    const double duration = step.end_time - step.start_time;
    tick_count_for_current_step =
      std::max(1, static_cast<int>(duration / Joint::kControlDt));

    get_delta_steps(step.start_position, step.end_position, tick_count_for_current_step);
    
    last_step_index = current_step_index;
  }

  last_sent_position = step_toward(last_sent_position, step.end_position);

  const bool on_last = (current_step_index + 1 == steps.size());
  if (on_last && time_seconds >= step.end_time &&
      all_reached(last_sent_position, final_position, kFinalPositionTolerance)) {
    return std::nullopt;
  }

  return last_sent_position;
}

bool Interpolator::is_done(double time_seconds) const
{
  return time_seconds >= total_duration &&
    all_reached(last_sent_position, final_position, kFinalPositionTolerance);
}

bool Interpolator::all_reached(
  const std::vector<double>& current,
  const std::vector<double>& target,
  double tolerance) const
{
  if (current.size() != target.size()) {
    return false;
  }
  for (std::size_t i = 0; i < current.size(); i++) {
    if (std::abs(current[i] - target[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

const std::vector<double>& Interpolator::end_position() const
{
  return final_position;
}

void Interpolator::get_delta_steps(
  const std::vector<double>& start,
  const std::vector<double>& target,
  const int tick_count)
{
  delta_steps.assign(start.size(), 0.0);
  for (std::size_t i = 0; i < start.size(); i++) {
    delta_steps[i] = (target[i] - start[i]) / tick_count;
  }
}

std::vector<double> Interpolator::step_toward(
  const std::vector<double>& current,
  const std::vector<double>& target) const
{
  std::vector<double> result(current.size());
  for (std::size_t i = 0; i < current.size(); i++) {
    const double clamped_step =
      std::clamp(delta_steps[i], -Joint::kMaxJointDelta, Joint::kMaxJointDelta);
    double next = current[i] + clamped_step;

    if ((target[i] - current[i]) * (target[i] - next) < 0.0) {
      next = target[i];
    }
    result[i] = next;
  }
  return result;
}

}  // namespace booster_controller