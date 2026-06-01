#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <booster_interface/msg/motor_state.hpp>
#include <booster_interface/msg/low_cmd.hpp>

class IController {
public:
  virtual void init(rclcpp::Node::SharedPtr node) = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    booster_interface::msg::LowCmd& command) = 0;

  virtual ~IController() = default;
};