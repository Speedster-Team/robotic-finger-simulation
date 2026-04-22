#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class FingerControl : public rclcpp::Node
{
public:
  FingerControl()
  : Node("finger_control"), count_(0)
  {
    // create publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("sub", 10);

    // define timer callback and init
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = {0.1, 0.1, 0.1};
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerControl>());
  rclcpp::shutdown();
  return 0;
}