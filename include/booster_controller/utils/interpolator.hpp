#pragma once

#include <optional>
#include <vector>

#include "action_interface/msg/joint_trajectory.hpp"
#include "booster_controller/utils/joint_command.hpp"

namespace booster_controller
{

struct Step {
  double start_time;
  double end_time;
  std::vector<double> start_position;
  std::vector<double> end_position;
};

class Interpolator {
public:
  void load(const action_interface::msg::JointTrajectory& trajectory);
  std::optional<std::vector<double>> sample(double time_seconds);
  bool is_done(double time_seconds) const;
  const std::vector<double>& end_position() const;
  void set_start_positions(const std::vector<double>& current_joint_position);
  bool all_reached(
    const std::vector<double>& current,
    const std::vector<double>& target,
    double tolerance) const;

private:
  std::vector<Step> steps;
  std::vector<double> final_position;
  std::vector<double> last_sent_position;
  std::size_t current_step_index{0};

  std::vector<double> step_toward(
      const std::vector<double>& current,
      const std::vector<double>& target);
    double total_duration = 0.0;

  static std::vector<double> lerp(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double alpha);
};

}  // namespace booster_controller
