#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialStream.h>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "adt_code_msg/srv/arduino_command.hpp"

using namespace std::chrono_literals;
using namespace LibSerial;

class ArduinoBridge : public rclcpp::Node
{
  public:
    LibSerial::SerialStream serial_port;
  
    ArduinoBridge()
    : Node("arduino_bridge")
    {
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        std::string serial_port_device = this->get_parameter("serial_port").as_string();
        
        this->declare_parameter("command_topic", "command");
        std::string command_topic = this->get_parameter("command_topic").as_string();

        serial_port.Open( serial_port_device );

        serial_port.SetBaudRate( BaudRate::BAUD_115200 );
        serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
        serial_port.SetParity( Parity::PARITY_NONE );
        serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;

    
        command_srv_ = this->create_service<adt_code_msg::srv::ArduinoCommand>(command_topic, std::bind(&ArduinoBridge::command_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    bool send_command(const std::string & command)
    {
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
            RCLCPP_INFO(this->get_logger(), "Type: Event");
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
  
    void command_callback(
        const std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Request> request,
        std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
    }
    rclcpp::Service<adt_code_msg::srv::ArduinoCommand>::SharedPtr command_srv_;
  

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoBridge>());
  rclcpp::shutdown();
  return 0;
}
