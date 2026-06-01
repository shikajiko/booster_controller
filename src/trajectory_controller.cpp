#include "booster_controller/joint_trajectory_controller.hpp"

using Action     = TrajectoryController::Action;
using GoalHandle = TrajectoryController::GoalHandle;

void TrajectoryController::init(rclcpp::Node::SharedPtr node) 
{
  this.node = node;
  action_server = rclcpp_action::create_server<Action>(
    node,
    "controller/run_trajectory",
    std::bind(&JointTrajectoryController::handle_goal,     this, _1, _2),
    std::bind(&JointTrajectoryController::handle_cancel,   this, _1),
    std::bind(&JointTrajectoryController::handle_accepted, this, _1));
}

void TrajectoryController::activate() 
{
  next_command_target.reserve(Joint::kJointCnt);
  prev_joint_states.reserve(Joint::kJointCnt);
  executing = true;
}

void TrajectoryController::deactivate() {
  if (active_goal) {
      abort_active_goal(
          Action::Result::ERROR_INVALID_GOAL,
          "Controller deactivated mid-trajectory");
  }

  next_command_target.clear();
  prev_joint_states.clear();

  executing = false;
  elapsed   = 0.0;
}

rclcpp_action::GoalResponse TrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const Action::Goal> goal)
{
  if (active_goal) {
    RCLCPP_WARN(node_->get_logger(),
    "Action rejected, another action is still running");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!executing) {
      RCLCPP_WARN(node_->get_logger(),
      "Action rejected, joint trajectory is not executing");
      return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty()) {
      RCLCPP_WARN(node->get_logger(),
      "Action rejected, trajectory is empty");
      return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryController::handle_cancel(
    std::shared_ptr<GoalHandle> goal_handle)
{
    //TO DO: return the robot into safe position
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryController::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) 
{
  try {
    interpolator.load(goal_handle->get_goal()->trajectory);
  } catch (const std::exception& e) {
    auto result = std::make_shared<Action::Result>();
    result->error_code   = Action::Result::ERROR_INVALID_GOAL;
    result->error_string = e.what();
    goal_handle->abort(result);
    return;
}
  active_goal = goal_handle;
  elapsed     = 0.0;
}

void TrajectoryController::update(
    double dt,
    const std::vector<double>& current_joint_q,
    booster_interface::msg::LowCmd& command)
{
    if (!active_goal) {
        return;
    }

    if (elapsed == 0.0) 
    {
      interpolator.set_start_positions(current_joint_q);
      update_prev_joint(current_joint_q);
    }

    elapsed += dt;
    auto positions = interpolator.sample(elapsed);

    command = construct_joint_command(prev_joint_states, next_command_target);
    update_prev_joint(positions);

    if (!positions) {
        auto end_pos = interpolator.end_position();
        update_command_target(end_pos);
        command = construct_joint_command(prev_joint_states, next_command_target);

        auto result = std::make_shared<Action::Result>();
        result->error_code   = 0;
        result->error_string = "OK";
        active_goal->succeed(result);
        active_goal = nullptr;
        elapsed     = 0.0;
        return;
    }

    auto feedback = std::make_shared<Action::Feedback>();
    feedback->has_run_for_seconds = elapsed;
    feedback->desired.positions = *positions;
    feedback->actual.positions = current.position;

    active_goal->publish_feedback(feedback);
}

void TrajectoryController::abort_active_goal(
    int32_t error_code, const std::string& reason)
{
    if (!active_goal) return;
    auto result = std::make_shared<Action::Result>();
    result->error_code   = error_code;
    result->error_string = reason;
    active_goal->abort(result);
    active_goal = nullptr;
    elapsed     = 0.0;
}

void TrajectoryController::update_prev_joint(const std::vector<double> & position) 
{
  for (size_t i = 0; i < position.size(); i++) {
    prev_joint_states[i].q = position[i];
  }
}

void TrajectoryController::update_command_target(const std::vector<double> & position) 
{
  for (size_t i = 0; i < position.size(); i++) {
    next_command_target[i].q = position[i];
  }
}

bool TrajectoryController::is_idle() 
{
  if (active_goal) {
    return false;
  }

  return true;
}