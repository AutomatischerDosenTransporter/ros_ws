#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adt_code_msg/srv/relay_switch.hpp"
#include <libserial/SerialStream.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace LibSerial;

class BasisRoboterRelayDriver : public rclcpp::Node
{
  public:
    LibSerial::SerialStream serial_port;
  
    BasisRoboterRelayDriver()
    : Node("basis_roboter_relay_driver")
    {
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        std::string serial_port_device = this->get_parameter("serial_port").as_string();

        serial_port.Open( serial_port_device );

        serial_port.SetBaudRate( BaudRate::BAUD_115200 );
        serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
        serial_port.SetParity( Parity::PARITY_NONE );
        serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;

        relay_switch_server_ = this->create_service<adt_code_msg::srv::RelaySwitch>("relay_switch", std::bind(&BasisRoboterRelayDriver::relaySwitchCallback, this, std::placeholders::_1,  std::placeholders::_2));


    }

  private:
    bool send_command(const std::string & command) {
      std::stringstream ss;
      ss << command;
      serial_port << ss.str() << std::endl ;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", command.c_str());

      std::string response;
      while (true) {
        try
        {
          std::getline(serial_port, response);
          RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());

          if(response.rfind("$ OK", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: OK");
            return true;
          }  else if (response.rfind("$ ERROR", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Error");
            return false;
          } else if (response.rfind("+", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Message");
          } else if (response.rfind("&", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Message");
          } else if (response.rfind("#", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Echo");
          } else {
            RCLCPP_INFO(this->get_logger(), "Type: Unknown");
            return false;
          }
        }
        catch (const LibSerial::ReadTimeout&)
        {
          RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
          return false;
        }
      }
    }

    void relaySwitchCallback(
        const std::shared_ptr<adt_code_msg::srv::RelaySwitch::Request> request,
        std::shared_ptr<adt_code_msg::srv::RelaySwitch::Response> response)
    {

        if(request->on) {
          std::string cmd = "on -number " + std::to_string(request->number);
          response->ok = send_command(cmd);
        } else {
          std::string cmd = "off -number " + std::to_string(request->number);
          response->ok = send_command(cmd);
        }
        RCLCPP_INFO(this->get_logger(), "Incoming request state: %d" " number: %ld", request->on, request->number);
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
