#include "booster_joint_manager/joint.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"

#include <cstdlib>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace booster_joint_manager {

class ReadJointProvider
{
public:
ReadJointProvider(
  rclcpp::Node::SharedPtr node,
  bool print_all,
  std::vector<Joint::JointIndex> joints)
  : node(node), print_all(print_all), joints(joints)
{
  low_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
    "/low_state",
    10,
    [this](booster_interface::msg::LowState::SharedPtr msg) {
        if (this->print_all) {
          print_all_joint_info(msg->motor_state_parallel);
        } else {
          for (auto joint : this->joints) {
            const auto index = Joint::joint_to_index(joint);
            if (index < msg->motor_state_parallel.size()) {
              print_joint_info(joint, msg->motor_state_parallel[index]);
            }
          }
        }
      });
}

private:
rclcpp::Node::SharedPtr node;
bool print_all;
std::vector<Joint::JointIndex> joints;
rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr low_state_subscriber;
void print_joint_info(Joint::JointIndex joint, booster_interface::msg::MotorState state);
void print_all_joint_info(const std::vector<booster_interface::msg::MotorState> & states);
};

void ReadJointProvider::print_joint_info(
  Joint::JointIndex joint,
  booster_interface::msg::MotorState state)
{
  RCLCPP_INFO(
    node->get_logger(),
    "joint %d (%.*s): q=%.4f dq=%.4f ddq=%.4f tau_est=%.4f",
    static_cast<int>(joint),
    static_cast<int>(Joint::joint_name(joint).size()),
    Joint::joint_name(joint).data(),
    state.q,
    state.dq,
    state.ddq,
    state.tau_est);
}

void ReadJointProvider::print_all_joint_info(const std::vector<booster_interface::msg::MotorState> & states)
{
  for (const auto joint : Joint::kAllJoints) {
    const auto index = Joint::joint_to_index(joint);
    if (index < states.size()) {
      print_joint_info(joint, states[index]);
    }
  }
}

}

int main(int argc, char * argv[])
{

  bool all_joint = argc == 1? true : false;
  std::vector<booster_joint_manager::Joint::JointIndex> joints;
  for (int i = 1; i < argc; ++i) {
    const auto joint = std::atoi(argv[i]);
    if (joint >= 0 &&
        static_cast<std::size_t>(joint) < booster_joint_manager::Joint::kJointCnt)
    {
      joints.push_back(static_cast<booster_joint_manager::Joint::JointIndex>(joint));
    }
  }
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("read_joint_node");
  const booster_joint_manager::ReadJointProvider read_joint_provider(node, all_joint, joints);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
