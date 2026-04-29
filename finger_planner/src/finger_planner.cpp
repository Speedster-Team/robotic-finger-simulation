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

#include "rclcpp/rclcpp.hpp"

#include "finger_interfaces/srv/send_command.hpp"
#include "finger_interfaces/srv/waypoints.hpp"

using namespace std::chrono_literals;

class FingerPlanner : public rclcpp::Node
{
public:
  FingerPlanner()
  : Node("finger_planner")
  {
    // create client
    send_client_ = create_client<finger_interfaces::srv::SendCommand>("/send_service");

    
    // define service callback function
    auto send_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::Waypoints::Request> request,
      std::shared_ptr<finger_interfaces::srv::Waypoints::Response> response) -> void
      {

        // compute trajectories and save
        for (auto i = 0; i < request->length; i++) {
            // request->x[i];
            // request->y[i];
            // request->z[i];
        }

        // splice trajectories together
        // auto msg = finger_interfaces::srv::SendCommand();

        // send trajectories
        // send_client_->async_send_request(msg);
        
        // simulation will always recieve command
        response->success = 1;
      };

    // create service
    move_service_ = create_service<finger_interfaces::srv::Waypoints>("/move_finger",
      send_service_callback);

  }

private:
  rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedPtr send_client_;
  rclcpp::Service<finger_interfaces::srv::Waypoints>::SharedPtr move_service_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerPlanner>());
  rclcpp::shutdown();
  return 0;
}
