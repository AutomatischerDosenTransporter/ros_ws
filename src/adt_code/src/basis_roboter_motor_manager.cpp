#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BasisRoboterMotorManager : public rclcpp::Node
{
  public:  
    BasisRoboterMotorManager()
    : Node("basis_roboter_motor_manager")
    {
         this->declare_parameter("motor_driver_a_twist_topic", "/service_roboter/motor/a/command_twist");
          std::string motor_driver_a_twist_topic = this->get_parameter("motor_driver_a_twist_topic").as_string();

         this->declare_parameter("motor_driver_b_twist_topic", "/service_roboter/motor/b/command_twist");
          std::string motor_driver_b_twist_topic = this->get_parameter("motor_driver_b_twist_topic").as_string();

         this->declare_parameter("motor_manager_twist_topic", "/service_roboter/motor/manager/command_twist");
          std::string motor_manager_twist_topic = this->get_parameter("motor_manager_twist_topic").as_string();

        driver_a_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(motor_driver_a_twist_topic, 10);
        driver_b_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(motor_driver_b_twist_topic, 10);
        
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(motor_manager_twist_topic, 10, std::bind(&BasisRoboterMotorManager::twist_callback, this, _1));
    }
  
  private:
     void twist_callback(const geometry_msgs::msg::Twist & message)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: TWIST");
      driver_a_publisher_->publish(message);
      driver_b_publisher_->publish(message);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr driver_a_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr driver_b_publisher_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasisRoboterMotorManager>());
  rclcpp::shutdown();
  return 0;
}
