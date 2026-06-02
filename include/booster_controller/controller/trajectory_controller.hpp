#pragma once

#include <memory>
#include <vector>

#include "action_interface/action/action_trajectory.hpp"
#include "booster_controller/interface/controller_interface.hpp"
#include "booster_controller/utils/interpolator.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace booster_controller
{

class TrajectoryController : public IController {
public:
  using Action = action_interface::action::ActionTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void activate() override;
  void deactivate() override;
  void update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    booster_interface::msg::LowCmd& command) override;
  bool has_work() const override;
  bool is_idle() const;
  bool submit(std::shared_ptr<GoalHandle> goal_handle);
  bool cancel(std::shared_ptr<GoalHandle> goal_handle);

private:
  Interpolator interpolator;
  std::shared_ptr<GoalHandle> active_goal;
  double elapsed = 0.0;
  bool active = false;
  bool returning_to_stand = false;

  void load_stand_trajectory();
  static std::vector<double> positions_from_state(
    const std::vector<booster_interface::msg::MotorState>& current_states);
  static std::vector<JointCommandTarget> targets_from_positions(
    const std::vector<double>& positions);
};

}  // namespace booster_controller
