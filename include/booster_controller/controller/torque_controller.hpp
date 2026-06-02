#pragma once

#include <vector>

#include "booster_controller/interface/controller_interface.hpp"
#include "booster_controller/utils/joint_command.hpp"

namespace booster_controller
{

class TorqueController : public IController {
public:
  bool submit(const std::vector<Joint::JointIndex>& joints, bool enable_torque);
  void deactivate() override;
  void update(
    double dt,
    const std::vector<booster_interface::msg::MotorState>& current_states,
    booster_interface::msg::LowCmd& command) override;
  bool has_work() const override;

private:
  std::vector<Joint::JointIndex> pending_joints;
  bool pending_torque_enable = false;
  bool command_pending = false;
};

}  // namespace booster_controller
