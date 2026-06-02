#pragma once

#include <vector>

#include "booster_controller/controller_interface.hpp"
#include "booster_controller/interpolator.hpp"
#include "booster_controller/joint.hpp"
#include "booster_joint_interface/msg/transition_command.hpp"

namespace booster_joint_manager
{

class ModeTransitionController : public IController {
public:
  using TransitionCommand = booster_joint_interface::msg::TransitionCommand;

  bool submit_mode_switch(
    uint8_t target_mode,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    float& delay_second);
  bool submit_upper_body_control(
    bool enable,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    float& delay_second);
  void deactivate() override;
  void update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    booster_interface::msg::LowCmd& command) override;
  bool has_work() const override;

private:
  Interpolator interpolator;
  double elapsed = 0.0;
  bool command_running = false;

  bool load_target_positions(
    const std::vector<double>& target_positions,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    double duration_seconds);
  static std::vector<double> positions_from_state(
    const std::vector<booster_interface::msg::MotorState>& current_states);
  static std::vector<JointCommandTarget> targets_from_positions(
    const std::vector<double>& positions);
};

}  // namespace booster_joint_manager
