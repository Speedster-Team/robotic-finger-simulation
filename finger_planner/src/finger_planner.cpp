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
#include "finger_interfaces/action/sinusoidal.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

/// \brief State variable for the command sent to the finger
enum CmdState
{
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
  using GoalHandleSinusoidal = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Sinusoidal>;

  FingerPlanner()
  : Node("finger_planner"),
    ra_ (0.0025), rb_ (0.0025), rc_ (0.0025),
    r1_ (0.008), r3_ (0.0045), r5_ (0.008), r7_ (0.0045), r9_ (0.009), r11_ (ra_ * 3.5),
    Ra_ {{ra_, 0, 0},          // splay
      {0, rb_, 0},             // mcp
      {0, 0, rc_}},            // pip/dip
    St_  {{r11_, -r3_, -r1_},  // splay joint
      {0, r7_, r5_},           // mcp joint
      {0, 0, r9_}},            // pip/dip joint
    slist_ {arma::vec6({0, 0, 1, 0, 0, 0}),
      arma::vec6({-1, 0, 0, 0, 0, 0.01776}),
      arma::vec6({-1, 0, 0, 0, 0, 0.07776}),
      arma::vec6({-1, 0, 0, 0, 0, 0.11836})},
    joint_min_ {-0.2, -0.2, -0.01},
    joint_max_ {0.2, 1.572, 1.572},
    M_ {{1, 0, 0, 0},
      {0, 1, 0, 0.16},
      {0, 0, 1, 0},
      {0, 0, 0, 1}},
    four_bar_lengths_ {8.83765 * 0.001,
      40.6 * 0.001,
      8.91536 * 0.001,
      37.79903 * 0.001},
    msg_attempts_ (0),
    cmd_state_ (CmdState::BEGIN)
  {
    // declare parameters
    auto param_desc1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc1.description = "The maximum joint velocity.";
    declare_parameter("max_velocity", 0.1, param_desc1);

    auto param_desc2 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc2.description = "The maximum joint acceleration.";
    declare_parameter("max_acceleration", 0.1, param_desc2);

    auto param_desc3 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc3.description = "The relative height of the ground (should be negative).";
    declare_parameter("relative_gnd_height", -0.25, param_desc3);

    // get parameters
    max_vel_ = get_parameter("max_velocity").as_double();
    max_accel_ = get_parameter("max_acceleration").as_double();
    gnd_height_ = get_parameter("relative_gnd_height").as_double();

    // create transformer class
    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_, joint_min_,
      joint_max_);
    generator_ = std::make_shared<JointTrajectory>(*transforms_, 100, gnd_height_);

    // create drake feedback subscription, forward as feedback
    auto motor_pos_sub_callback =
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) -> void {
        // save feedback
        motor_feedback_ = *msg;
      };

    motor_feedback_sub_ =
      create_subscription<finger_interfaces::msg::MotorFeedback>("/motor_pos_action_feedback",
      10, motor_pos_sub_callback);

    // create clients
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    send_client_ = create_client<finger_interfaces::srv::SendCommand>("/send_command", 10,
      send_cb_group_);
    start_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/start_command", 10,
      start_cb_group_);
    stop_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/stop_command", 10,
      stop_cb_group_);

    // wait for clients to appear
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

    // init callback groups for actions and action timers
    action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create cartesian move action
    auto cartesian_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Cartesian::Goal> goal)
      {
        // check that requested waypoints are same length
        if ((goal->length == int(goal->x.size())) && (goal->length == int(goal->y.size())) &&
          (goal->length == int(goal->z.size())))
        {
          // save waypoints as vector
          auto waypoints_temp = std::vector<arma::vec>();
          for (auto i = 0; i < goal->length; i++) {
            waypoints_temp.push_back({goal->x.at(i), goal->y.at(i), goal->z.at(i)});
          }

          // print request
          RCLCPP_INFO(get_logger(), "Received cartesian goal request with length %d and waypoints:",
          goal->length);

          // check that waypoints are within the joint limits
          try {
            RCLCPP_INFO_STREAM(get_logger(),
            "waypoint 0: (" << goal->x.at(0) << ", " << goal->y.at(0) << ", " << goal->z.at(0) <<
              ")");
            for(auto i = 1; i < goal->length; i++) {
              RCLCPP_INFO_STREAM(get_logger(),
              "waypoint " << i << ": (" << goal->x.at(i) << ", " << goal->y.at(i) << ", " <<
                goal->z.at(i) << ")");
              auto start = transforms_->end_effector_to_joint(waypoints_temp[i - 1]);
              auto end = transforms_->end_effector_to_joint(waypoints_temp[i]);
            }
          } catch (std::runtime_error & e) {
            RCLCPP_ERROR_STREAM(get_logger(),
            "Goal request REJECTED because waypoint is outside of joint limits!!");
            return rclcpp_action::GoalResponse::REJECT;
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
      const std::shared_ptr<GoalHandleCartesian>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel cartesian goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto cartesian_handle_accepted = [this](
      const std::shared_ptr<GoalHandleCartesian> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_cartesian_goal(goal_handle);
            },
        timer_cb_group_);
      };

    cartesian_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Cartesian>(
      this,
      "/cartesian_move",
      cartesian_handle_goal,
      cartesian_handle_cancel,
      cartesian_handle_accepted,
      rcl_action_server_get_default_options(),
      action_cb_group_);

    // create sinusoidal move action
    auto sinusoidal_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Sinusoidal::Goal> goal)
      {
        // check that joint is 0, 1, or 2
        // TODO: add more checks here?
        if (((goal->joint == 0) || (goal->joint == 1) || (goal->joint == 2))) {

          // print request
          RCLCPP_INFO_STREAM(get_logger(), "Received sinusoidal goal request for joint" <<
            goal->joint << " with amp " << goal->amp << ", freq " << goal->freq <<
            ", and v_shift " << goal->v_shift);

          // accept request
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          // print error
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because joint is not 0, 1, or 2.");

          // reject request
          return rclcpp_action::GoalResponse::REJECT;
        }
      };

    auto sinusoidal_handle_cancel = [this](
      const std::shared_ptr<GoalHandleSinusoidal>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel sinusoidal goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto sinusoidal_handle_accepted = [this](
      const std::shared_ptr<GoalHandleSinusoidal> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_sinusoidal_goal(goal_handle);
            },
        timer_cb_group_);
      };

    sinusoidal_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Sinusoidal>(
      this,
      "/sinusoidal_move",
      sinusoidal_handle_goal,
      sinusoidal_handle_cancel,
      sinusoidal_handle_accepted,
      rcl_action_server_get_default_options(),
      action_cb_group_);

  }

private:
  const double ra_, rb_, rc_;
  const double r1_, r3_, r5_, r7_, r9_, r11_;
  const arma::mat Ra_;
  const arma::mat St_;
  const std::vector<arma::vec6> slist_;
  const arma::vec joint_min_;
  const arma::vec joint_max_;
  const arma::mat44 M_;
  const std::vector<double> four_bar_lengths_;

  int msg_attempts_;

  CmdState cmd_state_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr motor_feedback_sub_;
  rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedPtr send_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr start_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr stop_client_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp_action::Server<finger_interfaces::action::Cartesian>::SharedPtr cartesian_action_server_;
  rclcpp_action::Server<finger_interfaces::action::Sinusoidal>::SharedPtr sinusoidal_action_server_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Result> cartesian_result_;
  std::shared_ptr<finger_interfaces::action::Sinusoidal::Result> sinusoidal_result_;
  finger_interfaces::msg::MotorFeedback motor_feedback_;

  double max_vel_;
  double max_accel_;
  double gnd_height_;
  std::vector<arma::vec> q_motor_list_;

  void execute_cartesian_goal(const std::shared_ptr<GoalHandleCartesian> goal_handle)
  {
    if (cmd_state_ == CmdState::IDLE) {
      // do nothing, wait for service callbacks to be called
      RCLCPP_INFO_ONCE(get_logger(), "idling...");

    } else if (cmd_state_ == CmdState::STOPPING) {
      RCLCPP_INFO(this->get_logger(), "Stopping action...");

      // cancel action timer now that its done
      action_timer_->cancel();

      // action has been stopped
      RCLCPP_INFO(this->get_logger(), "Action stopped.");

    } else if (cmd_state_ == CmdState::BEGIN) {
      RCLCPP_INFO(this->get_logger(), "Executing goal");

      const auto goal = goal_handle->get_goal();

      // initialize cartesian_result and set as success
      cartesian_result_ = std::make_shared<finger_interfaces::action::Cartesian::Result>();
      cartesian_result_->success = 1;

      // save waypoints as vector
      auto waypoints = std::vector<arma::vec>();
      for (int i = 0; i < goal->length; i++) {
        waypoints.push_back({goal->x.at(i), goal->y.at(i), goal->z.at(i)});
      }

      // generate motor trajectory
      try {
        q_motor_list_ = generator_->generate_cartesian(waypoints, max_vel_, max_accel_);

      } catch (std::runtime_error & e) {
        RCLCPP_INFO_STREAM(get_logger(), "Failed to generate motor trajectory.");

        // set result to failed
        cartesian_result_->success = 0;

        // abort action
        goal_handle->abort(cartesian_result_);

        // stop action timer now that its done
        cmd_state_ = CmdState::STOPPING;

      }

      // send trajectories
      auto rq = std::make_shared<finger_interfaces::srv::SendCommand::Request>();
      rq->length = int(q_motor_list_.size());
      rq->repeat = 0;

      for (auto & q : q_motor_list_) {
        rq->mcp_splay.push_back(q[0]);
        rq->mcp_flex.push_back(q[1]);
        rq->pip_flex.push_back(q[2]);
      }

      auto send_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedFutureWithRequest
        future)
        {
          if (future.get().second->success == 1) {
            // if successful, move to next stage
            cmd_state_ = CmdState::RECEIVED;

            // reset message attempt count
            msg_attempts_ = 0;

          } else {

            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'send' message.");

              // set result to failed
              cartesian_result_->success = 0;

              // abort action
              goal_handle->abort(cartesian_result_);

              // set state to cancelled to end timer
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to BEGIN, try sending again
              cmd_state_ = CmdState::BEGIN;
            }

          }
        };

      // send request
      auto result_future = send_client_->async_send_request(rq, send_client_callback);

      // idle while we wait for a response
      cmd_state_ = CmdState::IDLE;

      RCLCPP_INFO(this->get_logger(), "Send request sent goal");

    } else if (cmd_state_ == CmdState::RECEIVED) {
      RCLCPP_INFO(this->get_logger(), "Data recieved");

      auto start_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::
        SharedFutureWithRequest future) {
          if (future.get().second->success == 1) {
            RCLCPP_INFO(this->get_logger(), "Starting");

            // if successful, move to next stage
            cmd_state_ = CmdState::STARTED;

            // reset message attempt count
            msg_attempts_ = 0;

          } else {
            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'start' message.");

              // set result to failed
              cartesian_result_->success = 0;

              // abort action
              goal_handle->abort(cartesian_result_);

              // set state to cancelled to end timer
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to RECEIVED, try sending again
              cmd_state_ = CmdState::RECEIVED;
            }
          }
        };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = start_client_->async_send_request(rq, start_client_callback);

      // idle wait for response
      cmd_state_ = CmdState::IDLE;

    } else if (cmd_state_ == CmdState::STARTED) {
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "Control started...");

      // monitor feedback for stoppage
      if (motor_feedback_.active < 1e-4 && motor_feedback_.active > -1e-4) {
        RCLCPP_INFO_STREAM(get_logger(), "Control stopped, stopping action.");

        // if successful, safely stop
        cmd_state_ = CmdState::STOPPING;

        // assign result to handle
        goal_handle->succeed(cartesian_result_);

      } else {
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Working...");
      }

    } else if (cmd_state_ == CmdState::CANCELLED) {
      RCLCPP_INFO(this->get_logger(), "Cancelling action...");

      auto stop_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::
        SharedFutureWithRequest future) {
          if (future.get().second->success == 1) {
            // if successful, safely stop
            cmd_state_ = CmdState::STOPPING;

            // set result for cancellation
            cartesian_result_->success = 0;

            // assign result to handle
            goal_handle->canceled(cartesian_result_);

            // reset message attempts count
            msg_attempts_ = 0;

          } else {
            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'stop' message.");

              // set result to failed
              cartesian_result_->success = 0;

              // abort action
              goal_handle->abort(cartesian_result_);

              // stop action
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to CANCELLED, try sending again
              cmd_state_ = CmdState::CANCELLED;
            }
          }
        };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = stop_client_->async_send_request(rq, stop_client_callback);

      // idle while we wait for response
      cmd_state_ = CmdState::IDLE;
    }

    if (goal_handle->is_canceling()) {
      // begin action cancellation
      if ((cmd_state_ != CmdState::STOPPING) && (cmd_state_ != CmdState::IDLE)) {
        cmd_state_ = CmdState::CANCELLED;
      }
    }
  }

  void execute_sinusoidal_goal(const std::shared_ptr<GoalHandleSinusoidal> goal_handle)
  {
    if (cmd_state_ == CmdState::IDLE) {
      // do nothing, wait for service callbacks to be called
      RCLCPP_INFO_ONCE(get_logger(), "idling...");

    } else if (cmd_state_ == CmdState::STOPPING) {
      RCLCPP_INFO(this->get_logger(), "Stopping action...");

      // cancel action timer now that its done
      action_timer_->cancel();

      // action has been stopped
      RCLCPP_INFO(this->get_logger(), "Action stopped.");

    } else if (cmd_state_ == CmdState::BEGIN) {
      RCLCPP_INFO(this->get_logger(), "Executing goal");

      const auto goal = goal_handle->get_goal();

      // initialize cartesian_result and set as success
      sinusoidal_result_ = std::make_shared<finger_interfaces::action::Sinusoidal::Result>();
      sinusoidal_result_->success = 1;

      // generate motor trajectory
      try {
        q_motor_list_ = generator_->generate_sinusoid(goal->joint, goal->amp, goal->freq,
          goal->v_shift);

      } catch (std::runtime_error & e) {
        RCLCPP_INFO_STREAM(get_logger(), "Failed to generate motor trajectory.");

        // set result to failed
        sinusoidal_result_->success = 0;

        // abort action
        goal_handle->abort(sinusoidal_result_);

        // stop action timer now that its done
        cmd_state_ = CmdState::STOPPING;
      }

      // send trajectories
      auto rq = std::make_shared<finger_interfaces::srv::SendCommand::Request>();
      rq->length = int(q_motor_list_.size());
      rq->repeat = goal->repeat;

      for (auto & q : q_motor_list_) {
        rq->mcp_splay.push_back(q[0]);
        rq->mcp_flex.push_back(q[1]);
        rq->pip_flex.push_back(q[2]);
      }

      auto send_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedFutureWithRequest
        future)
        {
          if (future.get().second->success == 1) {
            // if successful, move to next stage
            cmd_state_ = CmdState::RECEIVED;

            // reset message attempt count
            msg_attempts_ = 0;

          } else {
            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'send' message.");

              // set result to failed
              sinusoidal_result_->success = 0;

              // abort action
              goal_handle->abort(sinusoidal_result_);

              // set state to cancelled to end timer
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to BEGIN, try sending again
              cmd_state_ = CmdState::BEGIN;
            }
          }
        };

      // send request
      auto result_future = send_client_->async_send_request(rq, send_client_callback);

      // idle while we wait for a response
      cmd_state_ = CmdState::IDLE;

      RCLCPP_INFO(this->get_logger(), "send request sent goal");

    } else if (cmd_state_ == CmdState::RECEIVED) {
      RCLCPP_INFO(this->get_logger(), "Data recieved");

      auto start_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::
        SharedFutureWithRequest future) {
          if (future.get().second->success == 1) {
            RCLCPP_INFO(this->get_logger(), "Starting");

            // if successful, move to next stage
            cmd_state_ = CmdState::STARTED;

            // reset message attempt count
            msg_attempts_ = 0;

          } else {
            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'start' message.");

              // set result to failed
              sinusoidal_result_->success = 0;

              // abort action
              goal_handle->abort(sinusoidal_result_);

              // set state to cancelled to end timer
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to RECEIVED, try sending again
              cmd_state_ = CmdState::RECEIVED;
            }
          }
        };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = start_client_->async_send_request(rq, start_client_callback);

      // idle we wait for response
      cmd_state_ = CmdState::IDLE;

    } else if (cmd_state_ == CmdState::STARTED) {
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "Control started...");

      // monitor feedback for stoppage
      if (motor_feedback_.active < 1e-4 && motor_feedback_.active > -1e-4) {
        RCLCPP_INFO_STREAM(get_logger(), "Control stopped, stopping action.");

        // if successful, safely stop
        cmd_state_ = CmdState::STOPPING;

        // assign result to handle
        goal_handle->succeed(sinusoidal_result_);

      } else {
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Working...");
      }
    } else if (cmd_state_ == CmdState::CANCELLED) {
      RCLCPP_INFO(this->get_logger(), "Cancelling action...");

      auto stop_client_callback =
        [this,
          goal_handle](rclcpp::Client<finger_interfaces::srv::StartStopCommand>::
        SharedFutureWithRequest future) {
          if (future.get().second->success == 1) {
            // if successful, safely stop
            cmd_state_ = CmdState::STOPPING;

            // set result for cancellation
            sinusoidal_result_->success = 0;

            // assign result to handle
            goal_handle->canceled(sinusoidal_result_);

            // reset message attempts count
            msg_attempts_ = 0;

          } else {
            // increment message attempts
            msg_attempts_++;

            if (msg_attempts_ >= 5) {
              RCLCPP_INFO_STREAM(get_logger(), "Failed to send 'stop' message.");

              // set result to failed
              sinusoidal_result_->success = 0;

              // abort action
              goal_handle->abort(sinusoidal_result_);

              // stop action
              cmd_state_ = CmdState::STOPPING;

            } else {
              // reset to CANCELLED, try sending again
              cmd_state_ = CmdState::CANCELLED;
            }
          }
        };

      // send request
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = stop_client_->async_send_request(rq, stop_client_callback);

      // idle wait for response
      cmd_state_ = CmdState::IDLE;
    }

    if (goal_handle->is_canceling()) {
      // begin action cancellation
      if ((cmd_state_ != CmdState::STOPPING) && (cmd_state_ != CmdState::IDLE)) {
        cmd_state_ = CmdState::CANCELLED;
      }
    }
  }
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
