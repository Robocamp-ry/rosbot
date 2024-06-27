#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode()
        : Node("motor_driver_node")
    {
        // Subscribe to the cmd_vel topic
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorDriverNode::cmd_vel_callback, this, std::placeholders::_1));

        // Initialize motor control pins or interfaces here
        // For example: setup GPIO pins, PWM interfaces, etc.
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract linear and angular velocities from the message
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        // Translate the velocity commands to motor control signals
        // Implement your motor control logic here

        double left_motor_speed = linear_velocity - angular_velocity;
        double right_motor_speed = linear_velocity + angular_velocity;

        // Control the motors with the calculated speeds
        // For example: set PWM values, send commands to motor driver, etc.
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
