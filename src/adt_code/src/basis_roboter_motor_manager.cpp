#include <cstdio>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BasisRoboterMotorManager : public rclcpp::Node
{
  public:  
    BasisRoboterMotorManager()
    : Node("basis_roboter_motor_manager")
    {
        

    }
  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasisRoboterMotorManager>());
  rclcpp::shutdown();
  return 0;
}
