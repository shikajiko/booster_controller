#include "booster_controller/utils/interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace booster_controller
{

void Interpolator::load(const action_interface::msg::JointTrajectory& trajectory)
{
  steps.clear();
  final_position.clear();
  last_sent_position.clear();
  total_duration = 0.0;

  if (trajectory.points.empty()) {
    throw std::invalid_argument("Trajectory has no points");
  }

  double cursor = 0.0;
  for (std::size_t i = 0; i < trajectory.points.size(); i++) {
    const auto& point = trajectory.points[i];
    if (point.positions.size() != Joint::kJointCnt) {
      throw std::invalid_argument("Number of joints does not match joint count");
    }

    std::vector<double> start_position =
      (i == 0) ? std::vector<double>(Joint::kJointCnt, 0.0)
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
  if (current_joint_position.size() != Joint::kJointCnt) {
    return;
  }

  if (!steps.empty()) {
    steps.front().start_position = current_joint_position;
    last_sent_position = current_joint_position;
  }
}

std::optional<std::vector<double>> Interpolator::sample(double time_seconds)
{
  for (const auto& step : steps) {
    if (time_seconds < step.start_time) {
      last_sent_position = step.start_position;
      return last_sent_position;
    }

    if (time_seconds <= step.end_time) {
      const double duration = step.end_time - step.start_time;
      const double alpha = duration <= 0.0
        ? 1.0
        : std::clamp((time_seconds - step.start_time) / duration, 0.0, 1.0);
      last_sent_position = lerp(step.start_position, step.end_position, alpha);
      return last_sent_position;
    }
  }

  if (!final_position.empty() &&
      !all_reached(last_sent_position, final_position, Joint::kMaxJointDelta)) {
    last_sent_position = lerp(last_sent_position, final_position, 1.0);
    return last_sent_position;
  }

  return std::nullopt;
}

bool Interpolator::is_done(double time_seconds) const
{
  return time_seconds >= total_duration &&
    all_reached(last_sent_position, final_position, Joint::kMaxJointDelta);
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

std::vector<double> Interpolator::lerp(
  const std::vector<double>& start,
  const std::vector<double>& end,
  double alpha)
{
  std::vector<double> result(start.size());
  for (std::size_t i = 0; i < start.size(); i++) {
    result[i] = start[i] + alpha * (end[i] - start[i]);
    result[i] = std::clamp(
      result[i],
      start[i] - Joint::kMaxJointDelta,
      start[i] + Joint::kMaxJointDelta);
  }
  return result;
}

}  // namespace booster_controller
