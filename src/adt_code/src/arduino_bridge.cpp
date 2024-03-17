#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

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
    // void command_listen;

    ArduinoBridge()
    : Node("arduino_bridge")
    {
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        std::string serial_port_device = this->get_parameter("serial_port").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter serial_port: '%s'", serial_port_device.c_str() );
        
        this->declare_parameter("command_service", "command");
        std::string command_service = this->get_parameter("command_service").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter command_service: '%s'", command_service.c_str() );
        
        this->declare_parameter("command_topic", "command");
        std::string command_topic = this->get_parameter("command_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter command_topic: '%s'", command_topic.c_str() );

        RCLCPP_INFO(this->get_logger(), "Starting serial connection");
        serial_port.Open( serial_port_device );

        serial_port.SetBaudRate( BaudRate::BAUD_115200 );
        serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
        serial_port.SetParity( Parity::PARITY_NONE );
        serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;
    
        command_srv_ = this->create_service<adt_code_msg::srv::ArduinoCommand>(command_service, std::bind(&ArduinoBridge::command_callback, this, std::placeholders::_1, std::placeholders::_2));
        serial_pub_ = this->create_publisher<std_msgs::msg::String>(command_topic, 10);
        std::thread command_listener_thread_([this](){ this->command_listen(); });
        RCLCPP_INFO(this->get_logger(), "Ready!");
    }

  private:
    bool send_command(const std::string & command)
    {
        std::stringstream ss;
        ss << command;
        serial_port << ss.str() << std::endl ;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", command.c_str());
    }
  
    void command_callback(
        const std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Request> request,
        std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
    }
    rclcpp::Service<adt_code_msg::srv::ArduinoCommand>::SharedPtr command_srv_;
    
    void command_listen()
    {
        RCLCPP_INFO(this->get_logger(), "Listenning on serial port");

        std::string response;
        while (true) {
            try
            {
                std::getline(serial_port, response);
                RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());

                auto message = std_msgs::msg::String();
                message.data = response;
                serial_pub_->publish(message);

                if(response.rfind("$ OK", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: OK");
                }  else if (response.rfind("$ ERROR", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Error");
                } else if (response.rfind("+", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Message");
                } else if (response.rfind("&", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Event");
                } else if (response.rfind("#", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Echo");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Type: Unknown");
                }
            }
            catch (const LibSerial::ReadTimeout&)
            {
                RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoBridge>());
  rclcpp::shutdown();
  return 0;
}
