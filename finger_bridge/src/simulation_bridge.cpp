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
#include "finger_interfaces/srv/start_stop_command.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

using namespace std::chrono_literals;

/// \brief State variable showing if data is ready to be sent to drake
enum State
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
    state_ (State::WAITING)
  {
    // define send service callback function
    auto send_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::SendCommand::Request> request,
      std::shared_ptr<finger_interfaces::srv::SendCommand::Response> response) -> void
      {
        RCLCPP_INFO(get_logger(), "send service request recieved...");

        // check that length field is filled
        if (request->length == 0) {
          response->success = 0;
          RCLCPP_ERROR(get_logger(), "send service request rejected, message field 'length' is 0.");
        } else if ((request->repeat != 0) && (request->repeat != 1)) {
          response->success = 0;
          RCLCPP_ERROR(get_logger(),
          "send service request rejected, message field 'request' is not 0 or 1.");
        } else {
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

          RCLCPP_INFO_STREAM(get_logger(), "send service request completed, response: " << int(response->success));
        }
      };
    // create callback group for send service
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create send service
    send_service_ = create_service<finger_interfaces::srv::SendCommand>("/send_command",
      send_service_callback, rclcpp::ServicesQoS(), send_cb_group_);

    // define start service callback function
    auto start_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::StartStopCommand::Request>,
      std::shared_ptr<finger_interfaces::srv::StartStopCommand::Response> response) -> void
      {
        RCLCPP_INFO(get_logger(), "start service request recieved...");

        // simulation will always recieve command
        response->success = 1;

        // make state ready
        state_ = State::READY;

        RCLCPP_INFO_STREAM(get_logger(), "start service request completed, response: " << int(response->success));
      };

    // create callback group for start service
    start_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create start service
    start_service_ = create_service<finger_interfaces::srv::StartStopCommand>("/start_command",
      start_service_callback, rclcpp::ServicesQoS(), start_cb_group_);

    // define stop service callback function
    auto stop_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::StartStopCommand::Request>,
      std::shared_ptr<finger_interfaces::srv::StartStopCommand::Response> response) -> void
      {
        RCLCPP_INFO(get_logger(), "stop service request recieved...");

        // simulation will always recieve command
        response->success = 1;

        // make state waiting
        state_ = State::WAITING;

        RCLCPP_INFO_STREAM(get_logger(), "stop service request completed, response: " << int(response->success));
      };

    // create callback group for stop service
    stop_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create stop service
    stop_service_ = create_service<finger_interfaces::srv::StartStopCommand>("/stop_command",
      stop_service_callback, rclcpp::ServicesQoS(), stop_cb_group_);

    // create publishers
    motor_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_position", 10);
    action_feedback_pub_ =
      create_publisher<finger_interfaces::msg::MotorFeedback>("/motor_pos_action_feedback", 10);

    // init motor feedback as zeros
    motor_feedback_.active = 0.0;
    motor_feedback_.motor_positions = {0.0, 0.0, 0.0};

    // define timer callback and init
    auto command_sender_timer_callback =
      [this]() -> void {
        // init count
        static auto count = 0;

        if (state_ == State::READY) {
          RCLCPP_INFO_ONCE(get_logger(), "publishing commands to drake...");

          // publish commands to drake
          auto msg = std_msgs::msg::Float64MultiArray();
          msg.data = {commands_.at(count).at(0), commands_.at(count).at(1),
            commands_.at(count).at(2)};
          motor_cmd_pub_->publish(msg);
          motor_feedback_.motor_positions = std::vector<float>(msg.data.begin(), msg.data.begin() + 2);

          // increment counter
          count++;

            // check for overflow
          if (count >= length_) {
            count = 0;
            if (repeat_ == 0) {
                    // disable control if no repeat
              state_ = State::WAITING;
            }
          }
        }

        // publish same as feedback
        motor_feedback_.active = (state_ == State::READY) ? 1.0 : 0.0;
        action_feedback_pub_->publish(motor_feedback_);
      };
    timer_ = this->create_wall_timer(10ms, command_sender_timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  State state_;
  finger_interfaces::msg::MotorFeedback motor_feedback_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
  rclcpp::Publisher<finger_interfaces::msg::MotorFeedback>::SharedPtr action_feedback_pub_;
  rclcpp::Service<finger_interfaces::srv::SendCommand>::SharedPtr send_service_;
  rclcpp::Service<finger_interfaces::srv::StartStopCommand>::SharedPtr start_service_;
  rclcpp::Service<finger_interfaces::srv::StartStopCommand>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;
  std::vector<std::vector<float>> commands_;
  int length_;
  int repeat_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<SimulationBridge>());
  auto node = std::make_shared<SimulationBridge>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
