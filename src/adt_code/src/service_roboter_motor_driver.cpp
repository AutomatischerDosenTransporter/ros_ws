#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialPort.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace LibSerial;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      this->declare_parameter("serial_port", "/dev/ttyACM0");
      this->declare_parameter("serial_baudrate", "9600");

      serial_port.Open( "/dev/ttyACM0" );
      serial_port.SetBaudRate( BaudRate::BAUD_9600 );

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      std::stringstream ss;
      ss << "speed -side " << "left" << " -speed " << "1.00" << "\r";
      message.data = ss.str();
      serial_port.FlushIOBuffers(); // Just in case
      serial_port.Write( ss.str());
      try
      {
        std::string response = "";
        serial_port.ReadLine(response, '\n', 0.5);
        RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());
        serial_port.ReadLine(response, '\n', 0.5);
        RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());
      }
      catch (const LibSerial::ReadTimeout&)
      {
        RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
      }



      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    SerialPort serial_port;

    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
