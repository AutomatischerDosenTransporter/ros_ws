#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace LibSerial;

class BasisRoboterMotorDriver : public rclcpp::Node
{
  public:
    LibSerial::SerialPort  serial_port;
  
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
    
        serial_pub_ = this->create_publisher<std_msgs::msg::String>("log", 100);
        encoder_x_pub_ = this->create_publisher<std_msgs::msg::Int32>("encoder/x", 1);
        encoder_y_pub_ = this->create_publisher<std_msgs::msg::Int32>("encoder/y", 1);
        encoder_z_pub_ = this->create_publisher<std_msgs::msg::Int32>("encoder/z", 1);

        limit_x_pos_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/x/positive", 10);
        limit_y_pos_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/y/positive", 10);
        limit_z_pos_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/z/positive", 10);
        
        limit_x_neg_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/x/negative", 10);
        limit_y_neg_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/y/negative", 10);
        limit_z_neg_pub_ = this->create_publisher<std_msgs::msg::Bool>("limit/z/negative", 10);

        serial_timer_ = this->create_wall_timer(30ms, std::bind(&BasisRoboterMotorDriver::command_listen, this));
        RCLCPP_INFO(this->get_logger(), "Ready!");
    }

  private:
    bool send_command(const std::string & command)
    {
        std::stringstream ss;
        ss << command;
        ss << std::endl;
        serial_port.Write(ss.str());
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", command.c_str());
        return true;
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

    void command_listen()
    {
        int timeout_ms = 25; // timeout value in milliseconds
        std::string response;
        try
            {
                serial_port.ReadLine( response, '\n', timeout_ms );
                response.erase (std::remove(response.begin(), response.end(), '\n'), response.end());
                response.erase (std::remove(response.begin(), response.end(), '\r'), response.end());


                bool log_response = true;
                if(response.rfind("$ OK", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: OK");
                }  else if (response.rfind("$ ERROR", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Error");
                } else if (response.rfind("+", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Message");
                } else if (response.rfind("&", 0) == 0) {
                    std::regex encoder_pattern("(\\& Encoder of [xyz] axis at [\\-]?[0-9]* stepps!)");
                    std::regex limit_set_pattern("(\\& Limit switch of [xyz] in [pnu] direction set!)");
                    std::regex limit_reset_pattern("(\\& Limit switch of [xyz] motor reset!)");

                    if(std::regex_match(response, encoder_pattern)) {
                        log_response = false;
                        try {
                            
                            response.erase(0, 13); //REMOVE '& Encoder of '
                            std::string axis = response.substr(0, 1); //EXTRACT 'x/y/z'
                            response.erase(0, 10); //REMOVE '[xyz] axis at '
                            response.erase(response.size()-8, 8);
                            int step = std::stoi(response);
                            auto step_message = std_msgs::msg::Int32();
                            step_message.data = step;
                            
                            if (axis.compare("x") == 0){
                                encoder_x_pub_->publish(step_message);}
                            if (axis.compare("y") == 0){
                                encoder_y_pub_->publish(step_message);}
                            if (axis.compare("z") == 0){
                                encoder_z_pub_->publish(step_message);}
                        } catch(...) {
                            RCLCPP_INFO(this->get_logger(), "Ecoder parsing failed: '%s'", response.c_str());
                        }

                    } else
                    if (std::regex_match(response, limit_set_pattern)) {
                        RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());
                        response.erase(0, 18); //REMOVE '& Limit switch of '
                        std::string axis = response.substr(0, 1); //EXTRACT 'x/y/z'
                        response.erase(0, 5); //REMOVE '[xyz]  in '
                        std::string direction_raw = response.substr(0, 1); //EXTRACT 'p/n/u'
                        bool direction_positive = direction_raw.compare("p");
                        bool direction_negative = direction_raw.compare("n");
                        bool direction_unknown = direction_raw.compare("u");

                        auto dir_message = std_msgs::msg::Bool();
                        dir_message.data = true;
                        
                        
                        if (axis.compare("x") == 0 && (direction_positive || direction_unknown)){
                            limit_x_pos_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }
                        if (axis.compare("y") == 0 && (direction_positive || direction_unknown)){
                            limit_y_pos_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }
                        if (axis.compare("z") == 0 && (direction_positive || direction_unknown)){
                            limit_z_pos_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }
                        
                        if (axis.compare("x") == 0 && (direction_negative || direction_unknown)){
                            limit_x_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }
                        if (axis.compare("y") == 0 && (direction_negative || direction_unknown)){
                            limit_y_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }
                        if (axis.compare("z") == 0 && (direction_negative || direction_unknown)){
                            limit_z_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send TOB");
                            }


                    } else
                    if (std::regex_match(response, limit_reset_pattern)) {
                        RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());

                        response.erase(0, 18); //REMOVE '& Encoder of '
                        std::string axis = response.substr(0, 1); //EXTRACT 'x/y/z'

                        auto dir_message = std_msgs::msg::Bool();
                        dir_message.data = false;
                        
                        if (axis.compare("x") == 0){
                            limit_x_pos_pub_->publish(dir_message);
                            limit_x_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send Kapa");
                            }
                        if (axis.compare("y") == 0){
                            limit_y_pos_pub_->publish(dir_message);
                            limit_y_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send Kapa");
                            }
                        if (axis.compare("z") == 0){
                            limit_z_pos_pub_->publish(dir_message);
                            limit_z_neg_pub_->publish(dir_message);
                            RCLCPP_INFO(this->get_logger(), "Send Kapa");
                            }

                        // RCLCPP_INFO(this->get_logger(), "Limit reset for : '%s'", axis.c_str());
                    }


                    if(log_response) RCLCPP_INFO(this->get_logger(), "Type: Event");
                } else if (response.rfind("#", 0) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Type: Echo");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Type: Unknown");
                }


                if(log_response) {
                    RCLCPP_INFO(this->get_logger(), "Response: '%s'", response.c_str());

                    auto message = std_msgs::msg::String();
                    message.data = response;
                    serial_pub_->publish(message);
                }
            }
            catch (const LibSerial::ReadTimeout&)
            {
                // RCLCPP_INFO(this->get_logger(), "The ReadByte() call has timed out.");
            }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_z_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_x_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_y_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_z_pos_pub_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_x_neg_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_y_neg_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limit_z_neg_pub_;

    rclcpp::TimerBase::SharedPtr serial_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasisRoboterMotorDriver>());
  rclcpp::shutdown();
  return 0;
}
