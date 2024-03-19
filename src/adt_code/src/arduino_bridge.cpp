#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <queue>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include "std_msgs/msg/string.hpp"
#include "adt_code_msg/srv/arduino_command.hpp"

using namespace std::chrono_literals;
using namespace LibSerial;

class ArduinoBridge : public rclcpp::Node
{
  public:
    LibSerial::SerialPort  serial_port;
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
    

        RCLCPP_INFO(this->get_logger(), "Serial port is not connected.");
        try {
            serial_port.Open( serial_port_device );
            serial_port.SetBaudRate( BaudRate::BAUD_115200 );
            serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
            serial_port.SetParity( Parity::PARITY_NONE );
            serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;
            RCLCPP_INFO(this->get_logger(), "Serial port connection succesfull.");
        } catch(const std::exception& ex) {
            RCLCPP_WARN(this->get_logger(),  "Serial port connection failed. Error: %s", ex.what());
            return;
        } catch(...) {
            RCLCPP_WARN(this->get_logger(), "Serial port connection failed with unknown exception.");
            return;
        }

        command_srv_ = this->create_service<adt_code_msg::srv::ArduinoCommand>(command_service, std::bind(&ArduinoBridge::command_callback, this, std::placeholders::_1, std::placeholders::_2));
        serial_pub_ = this->create_publisher<std_msgs::msg::String>(command_topic, 10);
        serial_timer_ = this->create_wall_timer(10ms, std::bind(&ArduinoBridge::command_listen, this));
        RCLCPP_INFO(this->get_logger(), "Ready!");
    }

  private:
    int command_queue_index = 0;
    int command_queue_counter = 0;

    bool send_command(const std::string & command)
    {
        std::stringstream ss;
        ss << command;
        ss << std::endl;
        serial_port.Write(ss.str());
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", command.c_str());
        return true;
    }
  
    void command_callback(
        const std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Request> request,
        std::shared_ptr<adt_code_msg::srv::ArduinoCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
        int queue_index = command_queue_counter++;

        while(command_queue_index < queue_index) {}
        command_queue_index++;

        send_command(request->request);

        response->succes = false;
        response->response = "something";
        RCLCPP_INFO(this->get_logger(), "Outgoing response");
    }
    rclcpp::Service<adt_code_msg::srv::ArduinoCommand>::SharedPtr command_srv_;
    
    void command_listen()
    {
        int timeout_ms = 25; // timeout value in milliseconds
        std::string response;
        try
            {
                serial_port.ReadLine( response, '\r', timeout_ms );
                RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());
                
                char bad_chars[] = "\n\r";
                for (unsigned int i = 0; i < strlen(bad_chars); ++i)
                {
                    // you need include <algorithm> to use general algorithms like std::remove()
                    response.erase (std::remove(response.begin(), response.end(), bad_chars[i]), response.end());
                }

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
                // RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
            }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoBridge>());
  rclcpp::shutdown();
  return 0;
}
