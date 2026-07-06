// trajectory_controller.hpp
#pragma once
#include <memory>
#include <optional>
#include <string_view>
#include <vector>
#include <booster_action_interface/action/action_trajectory.hpp>
#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/motor_state.hpp>
#include <booster_joint_interface/msg/set_joints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "booster_controller/interface/controller_interface.hpp"
#include "booster_controller/joint_state_manager/joint_state_manager.hpp"
#include "booster_controller/utils/joint_command.hpp"
#include "booster_controller/utils/interpolator.hpp"
#include "booster_joint_interface/msg/joint.hpp"

namespace booster_controller
{

class TrajectoryController : public IController
{
public:
  using Action = booster_action_interface::action::ActionTrajectory;
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
  void set_joint_state_manager(JointStateManager& manager);
  std::optional<booster_joint_interface::msg::SetJoints> gripper_command() const;

private:
  void load_stand_trajectory();
  void pad_gripper_positions(
    booster_action_interface::msg::JointTrajectory& trajectory) const;
  std::vector<double> positions_from_state(
    const std::vector<booster_interface::msg::MotorState>& current_states) const;
  std::vector<JointCommandTarget> targets_from_positions(
    const std::vector<double>& positions) const;

  bool active = false;
  double elapsed = 0.0;
  bool returning_to_stand = false;
  std::shared_ptr<GoalHandle> active_goal;
  Interpolator interpolator;
  JointStateManager* joint_state_manager = nullptr;
  std::optional<booster_joint_interface::msg::SetJoints> pending_gripper_command;
};

}  // namespace booster_controller