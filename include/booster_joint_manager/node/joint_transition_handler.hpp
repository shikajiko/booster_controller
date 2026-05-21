#include "booster_joint_manager/node/joint_manager_node.hpp"
#include "booster_joint_interface/srv/prepare_transition.hpp"
#include "booster_joint_interface/msg/transition_command.hpp"

namespace booster_joint_manager
{
class JointTransitionHandler
{
public:
    JointTransitionHandler(rclcpp::Node::SharedPtr node);
    void run_transition_service();

private:
    using JointPrepareService = booster_joint_interface::srv::PrepareTransition;
    using NextMode = booster_joint_interface::msg::TransitionCommand;
    rclcpp::Service<JointPrepareService>::SharedPtr joint_prepare_service;



}
}
