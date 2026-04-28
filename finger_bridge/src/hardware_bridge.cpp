#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "serial_interface/serial_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"

#include "finger_interfaces/srv/send_command.hpp"

using namespace std::chrono_literals;

class HardwareBridge : public rclcpp::Node
{
public:
  HardwareBridge()
  : Node("hardware_bridge"),
    serial_interface_ (std::make_shared<SerialInterface>())
  {
    // define service callback function
    auto send_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::SendCommand::Request> request,
      std::shared_ptr<finger_interfaces::srv::SendCommand::Response> response) -> void
      {

        // reformat
        std::vector<std::vector<float>> commands(request->length, std::vector<float>(3));
        for (int i = 0; i < request->length; i++) {
            commands[i] = {request->mcp_splay[i], request->mcp_flex[i], request->pip_flex[i]};
        }

        // send serial command
        serial_interface_->send_command(commands);

        // wait for result
        while (serial_interface_->get_message_status() == MessageStatus::NO_STATUS){
          //idle
          // std::cout << "idling" << std::endl;
        }

        // set return based on result
        if (serial_interface_->get_message_status() == MessageStatus::SUCCESS)
        {
          response->success = 1;
        }
        else {
          response->success = 0;
        }


      };

    // TODO: Check if this formulation is correct!!!! feels like cheating to use multithreaded executor
    // create callback group for service
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create service
    send_service_ = create_service<finger_interfaces::srv::SendCommand>("/send_service", send_service_callback, rclcpp::ServicesQoS(), send_cb_group_);

    // define timer callback and init
    auto timer_callback =
      [this]() -> void {
        serial_interface_->parse_response();

        if (serial_interface_->get_feedback_status() == FeedbackStatus::NEW_FEEDBACK) {
          auto fb = serial_interface_->get_feedback();
          for (auto f : fb){
            // for now, print feedback
            std::cout << float(f) << ' ';
          }
          std::cout << std::endl;
        }
      };
    timer_ = this->create_wall_timer(1ms, timer_callback);

    

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<SerialInterface> serial_interface_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Service<finger_interfaces::srv::SendCommand>::SharedPtr send_service_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<HardwareBridge>());
  auto node = std::make_shared<HardwareBridge>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}