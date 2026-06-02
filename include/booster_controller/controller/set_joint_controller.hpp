#pragma once

#include <vector>

#include "booster_controller/interface/controller_interface.hpp"
#include "booster_controller/utils/interpolator.hpp"
#include "booster_controller/utils/joint_command.hpp"

namespace booster_controller
{

class SetJointController : public IController {
public:
  bool submit(
    const std::vector<JointCommandTarget>& targets,
    const std::vector<booster_interface::msg::MotorState>& current_states);
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

  static std::vector<double> positions_from_state(
    const std::vector<booster_interface::msg::MotorState>& current_states);
  static std::vector<JointCommandTarget> targets_from_positions(
    const std::vector<double>& positions);
};

}  // namespace booster_controller
