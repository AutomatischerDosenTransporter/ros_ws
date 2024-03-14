#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialStream.h>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace LibSerial;

class BasisRoboterMotorDriver : public rclcpp::Node
{
  public:
    LibSerial::SerialStream serial_port;
  
    BasisRoboterMotorDriver()
    : Node("basis_roboter_motor_driver")
    {
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        this->declare_parameter("x_axis_scale", 1.0);
        this->declare_parameter("y_axis_scale", 1.0);
        this->declare_parameter("z_axis_scale", 1.0);
        std::string serial_port_device = this->get_parameter("serial_port").as_string();

        serial_port.Open( serial_port_device );

        serial_port.SetBaudRate( BaudRate::BAUD_115200 );
        serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
        serial_port.SetParity( Parity::PARITY_NONE );
        serial_port.SetStopBits( StopBits::STOP_BITS_1 ) ;

        command_subscription_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&BasisRoboterMotorDriver::command_callback, this, _1));
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("command_twist", 10, std::bind(&BasisRoboterMotorDriver::twist_callback, this, _1));
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

      double x = msg.linear.x;
      double y = msg.linear.y;
      double z = msg.linear.z;

      if(x >  1.0) x =  1.0;
      if(x < -1.0) x = -1.0;
      if(y >  1.0) y =  1.0;
      if(y < -1.0) y = -1.0;
      if(z >  1.0) z =  1.0;
      if(z < -1.0) z = -1.0;

      x *= 100.0 * this->get_parameter("x_axis_scale").as_double();
      y *= 100.0 * this->get_parameter("y_axis_scale").as_double();
      z *= 100.0 * this->get_parameter("z_axis_scale").as_double();

      std::string xData = "speed -axis x -speed " + std::to_string(x);
      std::string yData = "speed -axis y -speed " + std::to_string(y);
      std::string zData = "speed -axis z -speed " + std::to_string(z);
      
      send_command(xData);
      send_command(yData);
      send_command(zData);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasisRoboterMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
