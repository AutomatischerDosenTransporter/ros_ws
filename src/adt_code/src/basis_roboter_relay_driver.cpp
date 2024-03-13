#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adt_code_msg/srv/relay_switch.hpp"


using namespace std::chrono_literals;

class BasisRoboterRelayDriver : public rclcpp::Node
{
  public:
    BasisRoboterRelayDriver()
    : Node("basis_roboter_relay_driver")
    {
        relay_switch_server_ = this->create_service<adt_code_msg::srv::RelaySwitch>("relay_switch", std::bind(&BasisRoboterRelayDriver::relaySwitchCallback, this, std::placeholders::_1,  std::placeholders::_2));


    }

  private:
    void relaySwitchCallback(
        const std::shared_ptr<adt_code_msg::srv::RelaySwitch::Request> request,
        std::shared_ptr<adt_code_msg::srv::RelaySwitch::Response> response)
    {
        response->ok = true;
        RCLCPP_INFO(this->get_logger(), "Incoming request a: %d" " b: %ld", request->on, request->number);
        RCLCPP_INFO(this->get_logger(), "sending back response: [%d]", response->ok);
    }
    rclcpp::Service<adt_code_msg::srv::RelaySwitch>::SharedPtr relay_switch_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasisRoboterRelayDriver>());
  rclcpp::shutdown();
  return 0;
}
