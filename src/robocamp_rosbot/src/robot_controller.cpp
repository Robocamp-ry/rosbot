#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotController : public rclcpp::Node
{
public:
  RobotController()
  : Node("robot_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Robot Controller Node has been started.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
