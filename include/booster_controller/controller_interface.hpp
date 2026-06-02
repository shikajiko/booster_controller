#pragma once

#include <vector>

#include <booster_interface/msg/low_cmd.hpp>
#include <booster_interface/msg/motor_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace booster_joint_manager
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

  virtual ~IController() = default;
};

}  // namespace booster_joint_manager
