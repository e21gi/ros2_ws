// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_ros2/dxl.hpp"

using namespace std::chrono_literals;

class DynamixelPublisher : public rclcpp::Node
{
public:
    DynamixelPublisher() : Node("dynamixel_publisher"), count_(0)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
        timer_ = this->create_wall_timer(100ms, std::bind(&DynamixelPublisher::publish_velcmd_msg, this));
    }

private:
    void publish_velcmd_msg()
    {
        auto msg = geometry_msgs::msg::Twist();
        if (Dxl::kbhit())
        {
            char c = Dxl::getch();
            switch(c)
            {
            case 's': 
                msg.linear.x = 0; // left motor speed
                msg.linear.y = 0; // right motor speed        
                break;
            case 'f': 
                msg.linear.x = 200; // left motor speed
                msg.linear.y = -200; // right motor speed        
                break;
            case 'b': 
                msg.linear.x = -200; // left motor speed
                msg.linear.y = 200; // right motor speed        
                break;
            case 'l': 
                msg.linear.x = -200; // left motor speed
                msg.linear.y = -200; // right motor speed        
                break;
            case 'r': 
                msg.linear.x = 200; // left motor speed
                msg.linear.y = 200; // right motor speed        
                break;
            default:
                msg.linear.x = 0; // left motor speed
                msg.linear.y = 0; // right motor speed        
                break;
            }            
               
        RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf", msg.linear.x, msg.linear.y);
        dynamixel_publisher_->publish(msg);
        }        
        
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelPublisher>();
    std::cout << "Enter command(s,f,b,l,r):" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

