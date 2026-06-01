#pragma once

#include <memory>
#include <vector>
#include <string>

#include "booster_controller/controller_interface.hpp"
#include "booster_controller/interpolator.hpp"
#include "action_interface/action/action_trajectory.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "booster_controller/utils/command_constructor.hpp"
#include "booster_controller/joint.hpp"

namespace booster_joint_manager
{

class TrajectoryController : public IController {
public:
using Action = action_interface::action::ActionTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

void init(rclcpp::Node::SharedPtr node) override;
void activate()   override;
void deactivate() override;

void update(
    double dt,
    const std::vector<double>& current_joint_q,
    booster_interface::msg::LowCmd& command) override;

void update_prev_joint(const std::vector<double> & position);
void update_command_target(const std::vector<double> & position);

private:
rclcpp::Node::SharedPtr node;
rclcpp_action::Server<Action>::SharedPtr action_server;

Interpolator interpolator;
std::shared_ptr<GoalHandle> active_goal;
double elapsed = 0.0;
bool executing = false;

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const Action::Goal> goal);

rclcpp_action::CancelResponse handle_cancel(
  std::shared_ptr<GoalHandle> goal_handle);

void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);

void abort_active_goal(int32_t error_code, const std::string& reason);

std::vector<Joint::JointCommandTarget> next_command_target;
std::vector<booster_interface::msg::MotorState> prev_joint_states;
bool returning_to_stand = false;
};

}  // namespace booster_joint_manager
