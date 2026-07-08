#pragma once
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>
#include <booster_action_interface/msg/joint_trajectory.hpp>
#include "booster_controller/utils/joint_command.hpp"
#include "booster_model/joint.hpp"
namespace booster_controller
{
class Interpolator
{
public:
  void load(const booster_action_interface::msg::JointTrajectory& trajectory);
  void set_start_positions(const std::vector<double>& current_joint_position);
  std::optional<std::vector<double>> sample(double time_seconds);
  bool is_done(double time_seconds) const;
  const std::vector<double>& end_position() const;

private:
  struct Step
  {
    double start_time;
    double end_time;
    std::vector<double> start_position;
    std::vector<double> end_position;
  };

  bool all_reached(
    const std::vector<double>& current,
    const std::vector<double>& target,
    double tolerance) const;

  void get_delta_steps(
    const std::vector<double>& start,
    const std::vector<double>& target,
    int tick_count);

  std::vector<double> step_toward(
    const std::vector<double>& current,
    const std::vector<double>& target) const;

  std::vector<Step> steps;
  std::vector<double> final_position;
  std::vector<double> last_sent_position;
  std::vector<double> delta_steps;
  double total_duration = 0.0;
  std::size_t current_step_index = 0;
  std::size_t last_step_index = std::numeric_limits<std::size_t>::max();
  std::size_t joint_count = 0;
  int tick_count_for_current_step = 0;
};
}  // namespace booster_controller