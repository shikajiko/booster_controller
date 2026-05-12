#include <memory>

#include "booster_joint_manager/node/joint_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("joint_manager");
  const booster_joint_manager::JointManagerNode joint_manager_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
