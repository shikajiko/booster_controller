#pragma once
#include "booster_controller/controller_interface.hpp"
#include "booster_controller/interpolator.hpp"
#include "action_interface/action/action_trajectory.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

class TrajectoryController : public IController {
public:
  using Action       = action_interface::action::ActionTrajectory;
  using GoalHandle   = rclcpp_action::ServerGoalHandle<Action>;

  void init(rclcpp::Node::SharedPtr node) override;
  void activate()   override;
  void deactivate() override;

  void update(double dt,
              const sensor_msgs::msg::JointState& current,
              sensor_msgs::msg::JointState& command) override;

private:
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<Action>::SharedPtr action_server;

  Interpolator interpolator;
  std::shared_ptr<GoalHandle> active_goal;
  double elapsed = 0.0;
  bool   executing = false;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Action::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);

  void abort_active_goal(int32_t error_code, const std::string& reason);
};