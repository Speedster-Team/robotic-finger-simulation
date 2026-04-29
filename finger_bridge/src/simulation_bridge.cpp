/// \file
/// \brief bridges feedback and commands between ros and drake
///
/// PARAMETERS:
/// PUBLISHES:
///     torque_cmd (std_msgs::msg::Float64MultiArray): The torque commands for the motor.
/// SUBSCRIBES:
///     motor_position (std_msgs::msg::Float64MultiArray): The motor positions.
/// SERVERS:
///     send_service (finger_interfaces::srv::SendCommand): Gets command trajectories and saves to be output to drake.

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "finger_interfaces/srv/send_command.hpp"

using namespace std::chrono_literals;

/// \brief State variable showing if data is ready to be sent to drake
enum DataState
{
  READY,
  WAITING,
};

/// \brief A class that bridges commands and feedback between ros and drake
class SimulationBridge : public rclcpp::Node
{
public:
  /// \brief Create an instance of SimulationBridge
  SimulationBridge()
  : Node("simulation_bridge"),
    data_state_ (DataState::WAITING)
  {

    // create publishers
    motor_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/torque_cmd", 10);

    // create subscriptions
    auto motor_pos_sub_callback =
      [](std_msgs::msg::Float64MultiArray::UniquePtr msg) -> void {
        for (auto m : msg->data) {
            // for now, print feedback
          std::cout << float(m) << ' ';
        }
        std::cout << std::endl;
      };


    motor_feedback_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/motor_position",
      10, motor_pos_sub_callback);

    // define service callback function
    auto send_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::SendCommand::Request> request,
      std::shared_ptr<finger_interfaces::srv::SendCommand::Response> response) -> void
      {

        // save commands as vector
        commands_ = std::vector<std::vector<float>>(request->length, std::vector<float>(3));
        for (int i = 0; i < request->length; i++) {
          commands_[i] = {request->mcp_splay[i], request->mcp_flex[i], request->pip_flex[i]};
        }

        // save length and repeat
        length_ = request->length;
        repeat_ = request->repeat;

        // simulation will always recieve command
        response->success = 1;

        // flip data state
        data_state_ = DataState::READY;

      };

    // create service
    send_service_ = create_service<finger_interfaces::srv::SendCommand>("/send_service",
      send_service_callback);

    // define timer callback and init
    auto command_sender_timer_callback =
      [this]() -> void {
        // init count
        static auto count = 0;

        if (data_state_ == DataState::READY) {
            // publish commands to drake
          auto msg = std_msgs::msg::Float64MultiArray();
          msg.data = {commands_.at(count).at(0), commands_.at(count).at(1),
            commands_.at(count).at(2)};
          motor_cmd_pub_->publish(msg);

            // increment counter
          count++;

            // check for overflow
          if (count >= length_) {
            count = 0;
            if (repeat_ == 0) {
                    // disable control if no repeat
              data_state_ = DataState::WAITING;
            }
          }
        }
      };
    timer_ = this->create_wall_timer(10ms, command_sender_timer_callback);


  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  DataState data_state_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_feedback_sub_;
  rclcpp::Service<finger_interfaces::srv::SendCommand>::SharedPtr send_service_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  std::vector<std::vector<float>> commands_;
  int length_;
  int repeat_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationBridge>());
//   auto node = std::make_shared<HardwareBridge>();
//   rclcpp::executors::MultiThreadedExecutor exec;
//   exec.add_node(node);
//   exec.spin();
  rclcpp::shutdown();
  return 0;
}
