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
        static auto msg = geometry_msgs::msg::Twist();
        if (Dxl::kbhit())
        {
            char c = Dxl::getch();
            switch(c)
            {
            case 's':  //s를 누르면
                msg.linear.x = 0; //선속도
                msg.angular.z = 0; //각속도      
                break; //종료
            case 'p':  //p를 누르면
                msg.linear.x += 0.1; //선속도 0.1 증가
                break; //종료
            case 'm': //m를 누르면
                msg.linear.x -= 0.1; //선속도 0.1 감소
                break; //종료
            case 'o': //o를 누르면
                msg.angular.z += 0.1; //각속도 0.1 증가        
                break; //종료
            case 'n': //n를 누르면
                msg.angular.z -= 0.1; //각속도 0.1 감소
                break; //종료
            default:
                msg.linear.x = msg.linear.x; // 기존값 대입
                msg.angular.z = msg.angular.z; // 기존값     
                break;
            }            
               
        RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf", msg.linear.x, msg.angular.z);
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
    std::cout << "Enter command(s,p,m,o,n):" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

