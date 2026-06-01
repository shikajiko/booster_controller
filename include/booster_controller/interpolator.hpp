#pragma once

#include <cstddef>
#include <vector>
#include <optional>

#include "action_interface/msg/joint_trajectory.hpp"
#include "booster_joint_manager/joint.hpp"

namespace booster_joint_manager
{

struct Step {
  double t_start;
  double t_end;
  std::vector<double> from;
  std::vector<double> to;
};

class Interpolator {
public:
void load(const action_interface::msg::JointTrajectory& trajectory);
std::optional<std::vector<double> > sample(double t);
bool is_done(double t) const;
const std::vector<double>& end_position() const;
void set_start_positions(const std::vector<double>& current_joint_q);
bool all_reached(
  const std::vector<double>& current,
  const std::vector<double>& target,
  double tolerance) const;

private:
std::vector<Step> steps;
std::vector<double> end_position_;
std::vector<double> last_sent_position;
double total_duration = 0.0;

static std::vector<double> lerp(
  const std::vector<double>& a,
  const std::vector<double>& b,
  double alpha);
};

}  // namespace booster_joint_manager
