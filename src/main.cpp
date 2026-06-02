#include "booster_controller/node/controller_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("controller");
  const booster_controller::ControllerManagerNode controller_manager_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
