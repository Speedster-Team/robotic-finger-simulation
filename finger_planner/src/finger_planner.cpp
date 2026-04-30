/// \file
/// \brief generates trajectories and sends them to be executed
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
#include "finger_interfaces/srv/waypoints.hpp"
#include "finger_interfaces/action/cartesian.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

enum CmdState {
  IDLE,
  SENT,
  RECIEVED,
};

class FingerPlanner : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Cartesian>;

  FingerPlanner()
  : Node("finger_planner"),
    cmd_state_ (CmdState::IDLE)
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

    // create client
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    send_client_ = create_client<finger_interfaces::srv::SendCommand>("/send_service", 10, send_cb_group_);
    
    auto cartesian_handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const finger_interfaces::action::Cartesian::Goal> goal)
    {
      RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->length);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
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
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute_cartesian_goal(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    cartesian_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Cartesian>(
      this,
      "/cartesian_move",
      cartesian_handle_goal,
      cartesian_handle_cancel,
      cartesian_handle_accepted);
  
  }

private:
  CmdState cmd_state_;
  rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedPtr send_client_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::Service<finger_interfaces::srv::Waypoints>::SharedPtr cartesian_move_service_;
  rclcpp_action::Server<finger_interfaces::action::Cartesian>::SharedPtr cartesian_action_server_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Feedback> cartesian_feedback_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Result> cartesian_result_;

  void execute_cartesian_goal(const std::shared_ptr<GoalHandleCartesian> goal_handle) {
    if (cmd_state_ == CmdState::IDLE){
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(10);
      const auto goal = goal_handle->get_goal();
      cartesian_feedback_ = std::make_shared<finger_interfaces::action::Cartesian::Feedback>();
      cartesian_result_ = std::make_shared<finger_interfaces::action::Cartesian::Result>();

      // save waypoints as vector
      auto waypoints = std::vector<arma::vec>();
      for (int i = 0; i < goal->length; i++) {
        waypoints.push_back({goal->x.at(i), goal->y.at(i), goal->z.at(i)});
      }

      // generate motor trajectory
      auto q_motor_list = generator_->generate_cartesian(waypoints, 0.1, 0.1);

      // send trajectories
      auto rq = finger_interfaces::srv::SendCommand::Request::SharedPtr();
      
      for (auto q : q_motor_list) {
        rq->mcp_splay.push_back(q.at(0));
        rq->mcp_flex.push_back(q.at(1));
        rq->pip_flex.push_back(q.at(2));
      }

      // define callback function
      auto send_client_callback =
      [this](rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedFutureWithRequest future) {
        // update state
        cmd_state_ = CmdState::RECIEVED;

        // get result and assign to action result
        cartesian_result_->success = future.get().second->success;
        
        // assign action result
      };

      // send request
      auto result_future = send_client_->async_send_request(rq, send_client_callback);

      // update state
      cmd_state_ = CmdState::SENT;

    } else if (cmd_state_ == CmdState::SENT) {
      // get feedback and wait for response
      goal_handle->publish_feedback(cartesian_feedback_);
    } else if (cmd_state_ == CmdState::RECIEVED) {
      // Check if goal is done
      if (rclcpp::ok()) {
        goal_handle->succeed(cartesian_result_);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }

      // update cmd state for next time
      cmd_state_ = CmdState::IDLE;
    }
    

    
  };


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<FingerPlanner>());
  auto node = std::make_shared<FingerPlanner>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
