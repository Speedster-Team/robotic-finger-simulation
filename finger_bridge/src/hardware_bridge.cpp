/// \file
/// \brief bridges feedback and commands between ros and teensy
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVERS:
///     send_service (finger_interfaces::srv::SendCommand): Gets command trajectories and saves to be output to drake.

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "serial_interface/serial_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include "finger_interfaces/srv/send_command.hpp"
#include "finger_interfaces/srv/start_stop_command.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

/// \brief State variable showing if data is ready to be sent
enum State
{
  READY,
  WAITING,
};

class HardwareBridge : public rclcpp::Node
{
public:
  HardwareBridge()
  : Node("hardware_bridge"),
    serial_interface_ (std::make_shared<SerialInterface>()),
    state_ (State::WAITING)
  {
    // init motor feedback
    motor_feedback_.motor_positions = {0, 0, 0};
    motor_feedback_.active = 0.0;

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

          // reformat
          commands_ = std::vector<std::vector<float>>(request->length, std::vector<float>(3));
          for (int i = 0; i < request->length; i++) {
            commands_[i] = {request->mcp_splay[i], request->mcp_flex[i], request->pip_flex[i]};
          }

          // save length and repeat
          length_ = request->length;
          repeat_ = request->repeat;

          // send serial command
          serial_interface_->send_command(commands_, request->length, request->repeat);

          // wait for result
          while (serial_interface_->get_message_status() == MessageStatus::NO_STATUS) {
          }

          // set return based on result
          if (serial_interface_->get_message_status() == MessageStatus::SUCCESS) {
            response->success = 1;
          } else {
            response->success = 0;
          }

          RCLCPP_INFO(get_logger(), "send service request completed.");
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

        // send serial command
        serial_interface_->send_start();

        // wait for result
        while (serial_interface_->get_message_status() == MessageStatus::NO_STATUS) {
          //idle
          // std::cout << "idling" << std::endl;
        }

        // set return based on result
        if (serial_interface_->get_message_status() == MessageStatus::SUCCESS) {
          response->success = 1;
        } else {
          response->success = 0;
        }

        // make state ready
        state_ = State::READY;

        RCLCPP_INFO(get_logger(), "start service request completed.");
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

        // send serial command
        serial_interface_->send_stop();

        // wait for result
        while (serial_interface_->get_message_status() == MessageStatus::NO_STATUS) {
          //idle
          // std::cout << "idling" << std::endl;
        }

        // set return based on result
        if (serial_interface_->get_message_status() == MessageStatus::SUCCESS) {
          response->success = 1;
        } else {
          response->success = 0;
        }

        // make state waiting
        state_ = State::WAITING;

        RCLCPP_INFO(get_logger(), "stop service request completed.");
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

    // define feedback timer callback and init
    auto feedback_timer_callback =
      [this]() -> void {
        serial_interface_->parse_response();

        if (serial_interface_->get_feedback_status() == FeedbackStatus::NEW_FEEDBACK) {
          auto fb = serial_interface_->get_feedback();
          // for (auto f : fb) {
          //   // for now, print feedback
          //   // std::cout << float(f) << ' ';
          // }
          // std::cout << std::endl;

          motor_feedback_.motor_positions = std::vector<float>(fb.begin(), fb.begin() + 2);
          motor_feedback_.active = fb.at(3);
          action_feedback_pub_->publish(motor_feedback_);
        }

      };
    feedback_timer_ = this->create_wall_timer(1ms, feedback_timer_callback);

    // define timer callback and init
    auto command_sender_timer_callback =
      [this]() -> void {
        // init count
        static auto count = 0;

        if (state_ == State::READY) {
          RCLCPP_INFO_ONCE(get_logger(), "publishing commands to drake...");
          RCLCPP_INFO_STREAM(get_logger(), count);

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
              state_ = State::WAITING;
            }
          }
        }
      };
    teensy_sim_timer_ = this->create_wall_timer(10ms, command_sender_timer_callback);
  }

private:
  std::shared_ptr<SerialInterface> serial_interface_;
  State state_;
  finger_interfaces::msg::MotorFeedback motor_feedback_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr teensy_sim_timer_;
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
  // rclcpp::spin(std::make_shared<HardwareBridge>());
  auto node = std::make_shared<HardwareBridge>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
