#pragma once

#include <vector>

#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/motor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include "booster_joint_interface/msg/set_joints.hpp"

namespace booster_controller
{

class IController {
public:
  virtual void init(rclcpp::Node::SharedPtr node) {(void)node;}
  virtual void activate() {}
  virtual void deactivate() {}
  virtual void update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    booster_interface::msg::LowCmd& command) = 0;
  virtual bool has_work() const = 0;

  virtual std::optional<booster_joint_interface::msg::SetJoints> gripper_command() const
  {
    return std::nullopt;
  }

  virtual ~IController() = default;
};

}  // namespace booster_controller
