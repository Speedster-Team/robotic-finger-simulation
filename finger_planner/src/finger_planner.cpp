/// \file
/// \brief coordinates motion planning for finger movement.
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// SERVERS:
/// ACTIONS:

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "finger_interfaces/srv/send_command.hpp"
#include "finger_interfaces/srv/start_stop_command.hpp"
#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

/// \brief State variable for the command sent to the finger
enum CmdState {
  BEGIN,
  IDLE,
  RECEIVED,
  STARTED,
  CANCELLED,
  STOPPING,
};

class FingerPlanner : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Cartesian>;

  FingerPlanner()
  : Node("finger_planner"),
    cmd_state_ (CmdState::BEGIN)
  {

    // Radius Matrix
    const double ra = 0.0025; // splay
    const double rb = 0.0025; // mcp
    const double rc = 0.0025; // pip/dip

    const arma::mat Ra = {{ra, 0, 0}, // splay
                          {0, rb, 0}, // mcp
                          {0, 0, rc}}; // pip/dip


    // Structure Matrix
    const double r11 = ra*3.5;
    const double r1 = 8 * 0.001;
    //const double r2 = 8 * 0.001;
    const double r3 = 4.5 * 0.001;
    //const double r4 = 4.5 * 0.001;
    const double r5 = 8 * 0.001;
    //const double r6 = 8 * 0.001;
    const double r7 = 4.5 * 0.001;
    //const double r8 = 4.5 * 0.001;
    const double r9 = 9 * 0.001;
    //const double r10 = 9 * 0.001;

    const arma::mat St = {{r11, -r3, -r1}, //splay joint 
                          {0, r7, r5}, //mcp joint
                          {0, 0, r9}}; // pip/dip joint

    // screw axes (x = joint, y = finger, z = up) (origin is on splay joint)
    const std::vector<arma::vec6> slist = {
        arma::vec6({0,0,1,0,0,0}),
        arma::vec6({-1,0,0,0,0,0.01776}),
        arma::vec6({-1,0,0,0,0,0.07776}),
        arma::vec6({-1,0,0,0,0,0.11836})
    };

    const arma::vec joint_min = {-0.2, -0.2, -0.01};
    const arma::vec joint_max = {0.2, 1.572, 1.572};

    // very simple from onshape
    const arma::mat44 M = {{1, 0, 0, 0},
                          {0, 1, 0, 0.16},
                          {0, 0, 1, 0},
                          {0, 0, 0, 1}};

    // 4 bar lengths
    const std::vector<double> four_bar_lengths = {
        8.83765 * 0.001,
        40.6 * 0.001,
        8.91536 * 0.001,
        37.79903 * 0.001,
    };

    // create transformer class
    transforms_ = std::make_shared<Transformer>(Ra, St, slist, M, four_bar_lengths, joint_min, joint_max);
    generator_ = std::make_shared<JointTrajectory>(*transforms_, 100, -0.25);

    // create drake feedback subscription, forward as feedback
    auto motor_pos_sub_callback =
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) -> void {
        // save feedback
        motor_feedback_ = *msg;
      };
    motor_feedback_sub_ = create_subscription<finger_interfaces::msg::MotorFeedback>("/motor_pos_action_feedback",
      10, motor_pos_sub_callback);


    // create clients
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    send_client_ = create_client<finger_interfaces::srv::SendCommand>("/send_command", 10, send_cb_group_);
    start_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/start_command", 10, start_cb_group_);
    stop_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/stop_command", 10, stop_cb_group_);

    while (!send_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "client interrupted while waiting for send service to appear.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "waiting for send service to appear...");
    }
    while (!start_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "client interrupted while waiting for start service to appear.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "waiting for start service to appear...");
    }
    while (!stop_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "client interrupted while waiting for stop service to appear.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "waiting for stop service to appear...");
    }
    
    auto cartesian_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Cartesian::Goal> goal)
    {
      // check that requested waypoints are same length 
      if ((goal->length == int(goal->x.size())) && (goal->length == int(goal->y.size())) && (goal->length == int(goal->z.size()))) {
        // print request
        RCLCPP_INFO(get_logger(), "Received goal request with length %d and waypoints:", goal->length);
        for (auto i = 0; i < goal->length; i++) {
          RCLCPP_INFO_STREAM(get_logger(), "(" << goal->x.at(i) << ", " << goal->y.at(i) << ", " <<  goal->z.at(i) << ")");
        }

        // accept request
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      } else {
        // print error
        RCLCPP_INFO(get_logger(), "Goal request REJECTED because waypoints are malformed.");

        // reject request
        return rclcpp_action::GoalResponse::REJECT;
      }
    };

    auto cartesian_handle_cancel = [this](
      const std::shared_ptr<GoalHandleCartesian> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto cartesian_handle_accepted = [this](
      const std::shared_ptr<GoalHandleCartesian> goal_handle)
    {
      // start timer for action
      timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      action_timer_ = create_wall_timer(100ms, [this, goal_handle](){return this->execute_cartesian_goal(goal_handle);}, timer_cb_group_);
    };

    cartesian_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cartesian_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Cartesian>(
      this,
      "/cartesian_move",
      cartesian_handle_goal,
      cartesian_handle_cancel,
      cartesian_handle_accepted,
      rcl_action_server_get_default_options(),
      cartesian_cb_group_);
  
  }

private:
  CmdState cmd_state_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr motor_feedback_sub_;
  rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedPtr send_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr start_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr stop_client_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;  rclcpp::CallbackGroup::SharedPtr cartesian_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp_action::Server<finger_interfaces::action::Cartesian>::SharedPtr cartesian_action_server_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Feedback> cartesian_feedback_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Result> cartesian_result_;
  finger_interfaces::msg::MotorFeedback motor_feedback_;

  void execute_cartesian_goal(const std::shared_ptr<GoalHandleCartesian> goal_handle) {
    if (cmd_state_ == CmdState::IDLE) {
      // do nothing, wait for service callbacks to be called
      RCLCPP_INFO(get_logger(), "idling...");

    } else if (cmd_state_ == CmdState::BEGIN){
      RCLCPP_INFO(this->get_logger(), "Executing goal");

      const auto goal = goal_handle->get_goal();
      cartesian_feedback_ = std::make_shared<finger_interfaces::action::Cartesian::Feedback>();
      cartesian_result_ = std::make_shared<finger_interfaces::action::Cartesian::Result>();

      // initialize cartesian_result as success
      cartesian_result_->success = 1;
      
      // save waypoints as vector
      auto waypoints = std::vector<arma::vec>();
      for (int i = 0; i < goal->length; i++) {
        waypoints.push_back({goal->x.at(i), goal->y.at(i), goal->z.at(i)});
      }

      // generate motor trajectory
      auto q_motor_list = generator_->generate_cartesian(waypoints, 0.1, 0.1);

      // send trajectories
      auto rq = std::make_shared<finger_interfaces::srv::SendCommand::Request>();
      rq->length = int(q_motor_list.size());
      rq->repeat = 0;

      for (auto & q : q_motor_list) {
        rq->mcp_splay.push_back(q[0]);
        rq->mcp_flex.push_back(q[1]);
        rq->pip_flex.push_back(q[2]);
      }

      // define callback function
      auto send_client_callback =
      [this](rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedFutureWithRequest future) {
        // check result is success, handle errors here!
        if (future.get().second->success == 1) {
          // if successful, move to next stage
          cmd_state_ = CmdState::RECEIVED;

        } else {
          // reset to idle for now, this is incorrect
          cmd_state_ = CmdState::IDLE;
        }
        // TODO: handle other results
      };

      // send request
      auto result_future = send_client_->async_send_request(rq, send_client_callback);

      // update state
      cmd_state_ = CmdState::IDLE;

      RCLCPP_INFO(this->get_logger(), "send request sent goal");


    } else if (cmd_state_ == CmdState::RECEIVED) {
      RCLCPP_INFO(this->get_logger(), "Data recieved");

      // define callback function
      auto start_client_callback =
      [this](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedFutureWithRequest future) {
        // update state

        // check result is success, handle errors here!
        if (future.get().second->success == 1) {
          // if successful, move to next stage
          cmd_state_ = CmdState::STARTED;
          RCLCPP_INFO(this->get_logger(), "Starting");

        } else {
          // reset to idle for now, this is incorrect
          cmd_state_ = CmdState::IDLE;
        }
        // TODO: handle other results
      };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = start_client_->async_send_request(rq, start_client_callback);

      // update state while we wait for response
      cmd_state_ = CmdState::IDLE;

    } else if (cmd_state_ == CmdState::STARTED) {
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "Control started...");

      // monitor feedback for stoppage
      if (motor_feedback_.active < 1e-4) {
        RCLCPP_INFO_STREAM(get_logger(), "Control stopped, stopping action.");

        // define callback function
        auto stop_client_callback =
        [this, goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedFutureWithRequest future) {
          // check result is success, handle errors here!
          if (future.get().second->success == 1) {
            // if successful, safely stop
            cmd_state_ = CmdState::STOPPING;

            // set result for controller stopping
            cartesian_result_->success = 1;
            
            // assign result to handle
            if (rclcpp::ok()) {
              goal_handle->succeed(cartesian_result_);
              RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }

          } else {
            // reset to idle for now, this is incorrect
            cmd_state_ = CmdState::IDLE;

            // probably should crash here
          }
          // TODO: handle other results

        };

        // send request
        auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
        auto result_future = stop_client_->async_send_request(rq, stop_client_callback);

        // update state while we wait for response
        cmd_state_ = CmdState::IDLE;   
      } else {
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Working...");
      }

    } else if (cmd_state_ == CmdState::STOPPING) {
        RCLCPP_INFO(this->get_logger(), "Stopping cartesian action...");

        // cancel action timer now that its done
        action_timer_->cancel();

        // set state for next time
        cmd_state_ = CmdState::BEGIN;

        // action has been stopped
        RCLCPP_INFO(this->get_logger(), "Cartesian action stopped.");

    } else if (cmd_state_ == CmdState::CANCELLED) {
      RCLCPP_INFO(this->get_logger(), "Cancelling cartesian action...");

      // define callback function
      auto stop_client_callback =
      [this, goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedFutureWithRequest future) {
        // update state

        // check result is success, handle errors here!
        if (future.get().second->success == 1) {
          // if successful, safely stop
          cmd_state_ = CmdState::STOPPING;

          // set result for cancellation
          cartesian_result_->success = 55;

          // assign result to handle
          goal_handle->canceled(cartesian_result_);


        } else {
          // reset to idle for now, this is incorrect
          cmd_state_ = CmdState::IDLE;

          // probably should crash here
        }
        // TODO: handle other results

      };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = stop_client_->async_send_request(rq, stop_client_callback);

      // update state while we wait for response
      cmd_state_ = CmdState::IDLE;   
    }

    
    if (goal_handle->is_canceling()) {
      // begin action cancellation
      if (cmd_state_ != CmdState::STOPPING) {
        cmd_state_ = CmdState::CANCELLED;
      }
    }
  
  };


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerPlanner>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
