/// \file
/// \brief runs high level control coordinating perception and movement commands
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVERS:

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"

#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/action/sinusoidal.hpp"

using namespace std::chrono_literals;

/// \brief A class that bridges commands and feedback between ros and drake
class FingerControl : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Cartesian>;
  using GoalHandleSinusoidal = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Sinusoidal>;
  using Cartesian = finger_interfaces::action::Cartesian;
  using Sinusoidal = finger_interfaces::action::Sinusoidal;

  /// \brief Create an instance of SimulationBridge
  FingerControl()
  : Node("finger_control")
  {
    // wait for drake to startup
    wait_for_drake_heartbeat();

    // create action clients
    cartesian_client_ = rclcpp_action::create_client<Cartesian>(this,
      "/cartesian_move");
    sinusoidal_client_ = rclcpp_action::create_client<Sinusoidal>(this,
      "/sinusoidal_move");

    // wait for servers to appear
    while (!cartesian_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for service to appear...");
    }
    while (!cartesian_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for service to appear...");
    }

    // sleep for 1 second
    rclcpp::sleep_for(3000ms);

    // send test sinusoidal command
    send_sinusoid_goal(1, 1, 0.2, 1.0, 0.8);

    // send test cartesian command
    std::vector<float> start = {0, 0.15, -0.05};
    std::vector<float> end = {0, 0.08, -0.1};
    std::vector<std::vector<float>> waypoints = {start, end};
    send_cartesian_goal(waypoints);

    // go back now
    waypoints = {end, start};
    send_cartesian_goal(waypoints);
  }

private:
  rclcpp_action::Client<Cartesian>::SharedPtr cartesian_client_;
  rclcpp_action::Client<Sinusoidal>::SharedPtr sinusoidal_client_;

  void wait_for_drake_heartbeat()
  {
    // create dummy client to wait until drake is initialized
    auto heartbeat_client = create_client<std_srvs::srv::Empty>("/heartbeat", 10);

    // wait for server to appear
    while (!heartbeat_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for heartbeat service to appear...");
    }
  }

  void send_cartesian_goal(std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = finger_interfaces::action::Cartesian::Goal();

    // create goal request
    goal_msg.length = int(waypoints.size());
    for (auto & wp : waypoints) {
      goal_msg.x.push_back(wp.at(0));
      goal_msg.y.push_back(wp.at(1));
      goal_msg.z.push_back(wp.at(2));
    }

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Cartesian>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleCartesian::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleCartesian::SharedPtr,
      const std::shared_ptr<const Cartesian::Feedback>)
      {
        RCLCPP_INFO(get_logger(), "feedback recieved...");
      };

    send_goal_options.result_callback = [this](const GoalHandleCartesian::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = cartesian_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }

    // Now wait for the result
    auto result_future = cartesian_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal completed");
  }


  void send_sinusoid_goal(bool repeat, int joint, float amp, float freq, float v_shift)
  {
    auto goal_msg = finger_interfaces::action::Sinusoidal::Goal();

    // create goal request
    goal_msg.repeat = int(repeat);
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq = freq;
    goal_msg.v_shift = v_shift;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Sinusoidal>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleSinusoidal::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleSinusoidal::SharedPtr,
      const std::shared_ptr<const Sinusoidal::Feedback>)
      {
        RCLCPP_INFO(get_logger(), "feedback recieved...");
      };

    send_goal_options.result_callback = [this](const GoalHandleSinusoidal::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = sinusoidal_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }

    // Now wait for the result
    auto result_future = sinusoidal_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal completed");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerControl>());
//   auto node = std::make_shared<FingerControl>();
//   rclcpp::executors::MultiThreadedExecutor exec;
//   exec.add_node(node);
//   exec.spin();
  rclcpp::shutdown();
  return 0;
}
