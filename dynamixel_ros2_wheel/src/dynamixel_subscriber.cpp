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

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_ros2/dxl.hpp"

using std::placeholders::_1;

class DynamixelSubscriber : public rclcpp::Node
{
public:
    DynamixelSubscriber(): Node("dyanmixel_subscriber")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", qos_profile, std::bind(&DynamixelSubscriber::subscribe_topic_message, this, _1));
        
        if(!dxl.dxl_open()) RCLCPP_ERROR(this->get_logger(), "dynamixel open error");        
    }
    ~DynamixelSubscriber()
    {
        dxl.dxl_close();
    }
 
private:
    void subscribe_topic_message(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Received message: %lf,%lf", msg->linear.x,msg->angular.z);
        int r_rpm1=0, r_rpm2=0;
        dxl.convert_LARPM(msg->linear.x,msg->angular.z,&r_rpm1,&r_rpm2); //변환 함수 사용 dxl.cpp::242 참고
        std::cout<<"r_rpm1:"<<r_rpm1<<std::endl;
        std::cout<<"r_rpm2:"<<-r_rpm2<<std::endl;
        bool result = dxl.dxl_set_velocity(r_rpm1, -r_rpm2);
        if(result == false) 
        {
           RCLCPP_ERROR(this->get_logger(), "dxl_set_velocity error");     
        }         
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dynamixel_subscriber_;
    Dxl dxl;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


