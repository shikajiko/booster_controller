#include "booster_joint_manager/node/joint_transition_handler.hpp"

namespace booster_joint_manager
{

JointTransitionHandler::JointTransitionHandler(
  const rclcpp::Node::SharedPtr & node,
  JointManager & joint_manager)
: node(node), joint_manager(joint_manager)
{
  joint_prepare_service = node->create_service<JointPrepareService>(
    "prep_transition_service",
    std::bind(
      &JointTransitionHandler::parse_prepare_transition_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void JointTransitionHandler::parse_prepare_transition_request(
  const std::shared_ptr<JointPrepareService::Request> req,
  std::shared_ptr<JointPrepareService::Response> res)
{
  const auto & command = req->command;
  RCLCPP_INFO(
    node->get_logger(),
    "Received prep_transition_service request: transition=%u",
    static_cast<unsigned int>(command.transition));

  switch (command.transition) {
    case TransitionCommand::TRANSITION_MODE_SWITCH: {
        float delay_second = 0.0; 
        bool success = handle_mode_switch(command.target_mode, delay_second);
        res->success = success;
        res->delay_second = delay_second;
        return;
    }

    case TransitionCommand::TRANSITION_UPPER_BODY_CONTROL: {
        float delay_second = 0.0; 
        bool success = handle_upper_control_switch(command.upper_body_enable, delay_second);
        res->success = success;
        res->delay_second = delay_second;
      return;
    }

    default:
      res->success = false;
      res->delay_second = 0.;
      RCLCPP_WARN(
        node->get_logger(),
        "Rejected prep_transition_service request: unknown transition=%u",
        static_cast<unsigned int>(command.transition));
      return;
  }
}

bool JointTransitionHandler::handle_mode_switch(uint8_t target_mode, float & delay_second)
{
    switch (target_mode) {
        case TransitionCommand::MODE_STAND:
            delay_second = 2.5;
            return joint_manager.set_init_pose(0.0, 0.0);
        case TransitionCommand::MODE_WALK:
            delay_second = 0.1;
            return true;
        case TransitionCommand::MODE_CUSTOM:
            delay_second = 0.1;
            return joint_manager.set_init_pose(0.0, 0.5);
        
        default:
            return false;
    }
}

bool JointTransitionHandler::handle_upper_control_switch(bool enable, float & delay_second) {
    if (enable) {
        delay_second = 2.0;
        return joint_manager.set_init_arms(0.0, 0.5);
    } else {
        delay_second = 2.0;
        return joint_manager.set_init_arms(0.5, 0.0);
    }

    return false;
}

}  // namespace booster_joint_manager
