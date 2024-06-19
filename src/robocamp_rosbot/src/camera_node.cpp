#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class MJPGCameraPublisher : public rclcpp::Node {
public:
    MJPGCameraPublisher()
    : Node("mjpg_camera_publisher"), cap_("http://192.168.3.25:8000/stream.mjpg") {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening video stream or file");
        }

        publisher_ = image_transport::create_publisher(this, "camera/image");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // Approx 30 FPS
            std::bind(&MJPGCameraPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_.publish(msg);
        }
    }

    image_transport::Publisher publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MJPGCameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
