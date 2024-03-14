#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialStream.h>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace LibSerial;

class ServiceRoboterMotorDriver : public rclcpp::Node
{
  public:
    ServiceRoboterMotorDriver()
    : Node("service_roboter_motor_driver"), count_(0)
    {
      this->declare_parameter("serial_port", "/dev/ttyACM0");
      std::string serial_port_device = this->get_parameter("serial_port").as_string();

      serial_port.Open( serial_port_device );

      serial_port.SetBaudRate( BaudRate::BAUD_115200 );
      serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
      serial_port.SetParity( Parity::PARITY_NONE );
      serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;

      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // timer_ = this->create_wall_timer(500ms, std::bind(&ServiceRoboterMotorDriver::timer_callback, this));

      command_subscription_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&ServiceRoboterMotorDriver::command_callback, this, _1));
      twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&ServiceRoboterMotorDriver::twist_callback, this, _1));
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

    void command_callback(const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

      std::string data(msg.data.c_str());
      send_command(data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;


    void twist_callback(const geometry_msgs::msg::Twist & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: TWIST");

      double left = msg.linear.x * 0.5 + msg.angular.z * -0.5;
      double right = msg.linear.x * 0.5 + msg.angular.z * 0.5;

      std::string leftData = "speed -side left -speed " + std::to_string(left);
      std::string rightData = "speed -side right -speed " + std::to_string(right);
      
      send_command(leftData);
      send_command(rightData);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;


    void timer_callback()
    {
      std::stringstream ss;
      ss << "speed -side " << "left" << " -speed " << "1.00";
      serial_port << ss.str() << std::endl ;


      auto message = std_msgs::msg::String();
      message.data = ss.str();
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      std::string response;
      while (true) {
        try
        {
          std::getline(serial_port, response);
          RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());

          if(response.rfind("$ OK", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: OK");
            break;
          }  else if (response.rfind("$ ERROR", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Error");
            break;
          } else if (response.rfind("+", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Message");
          } else if (response.rfind("&", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Message");
          } else if (response.rfind("#", 0) == 0) {
            RCLCPP_INFO(this->get_logger(), "Type: Echo");
          } else {
            RCLCPP_INFO(this->get_logger(), "Type: Unknown");
          }
        }
        catch (const LibSerial::ReadTimeout&)
        {
          RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
          break;
        }
      }


    }


    rclcpp::TimerBase::SharedPtr timer_;
    SerialStream serial_port;

    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceRoboterMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
