#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("camera_node")
  {
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    }
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CameraNode::capture_and_publish, this));
  }

private:
  void capture_and_publish()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (!frame.empty()) {
      auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      image_publisher_->publish(*image_msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Captured empty frame!");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
