#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode()
  : Node("motor_driver_node")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MotorDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // Setup motor driver
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto linear_x = msg->linear.x;
    auto angular_z = msg->angular.z;

    // Send commands to motor driver
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}
