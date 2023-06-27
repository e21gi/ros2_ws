// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <cstdio>
#include <iostream>
#include <utility>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_ros2/camera.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher"), count_(1), width_(640), height_(360)
  {
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));
    qos_profile.reliability(reliability_policy);

    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos_profile);
    timer_ = this->create_wall_timer(30ms, std::bind(&CameraPublisher::publish_image, this));
    camera_publisher_pi = this->create_publisher<sensor_msgs::msg::Image>("image_pi", qos_profile);
    timer_pi = this->create_wall_timer(30ms, std::bind(&CameraPublisher::publish_image_pi, this));

    // webca
    cap_.open(1);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    // pi camera
    cap_pi.open(src_, cv::CAP_GSTREAMER);
    
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");        
    }

  }

private:
  void publish_image()
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;
    cap_ >> frame_;
    if (!frame_.empty()) {
        // Convert to a ROS image
        // cv::flip(frame, frame, 1); // Flip the frame if needed
        //convert_frame_to_message(frame_, count_, *msg);
        convert_frame_to_message(frame_, *msg);
        
        //cv::Mat cvframe = frame_;
        //cv::imshow("camimage", cvframe);
        //cv::waitKey(1);
        
        // Publish the image message and increment the frame_id.
        RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", count_++);
        camera_publisher_->publish(std::move(msg));       
    }
    else {
        RCLCPP_INFO(this->get_logger(), "frame empty");        
    }
  }
  void publish_image_pi()
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;
    cap_pi >> frame_pi;
    if (!frame_pi.empty()) {
        // Convert to a ROS image
        // cv::flip(frame, frame, 1); // Flip the frame if needed
        //convert_frame_to_message(frame_, count_, *msg);
        convert_frame_to_message(frame_pi, *msg);
        
        //cv::Mat cvframe = frame_;
        //cv::imshow("camimage", cvframe);
        //cv::waitKey(1);
        
        // Publish the image message and increment the frame_id.
        RCLCPP_INFO(this->get_logger(), "Publishing pi image #%zd", count_pi++);
        camera_publisher_pi->publish(std::move(msg));       
    }
    else {
        RCLCPP_INFO(this->get_logger(), "frame empty");        
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_pi;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_pi;
  size_t count_;
  size_t count_pi;
  cv::VideoCapture cap_;
  cv::VideoCapture cap_pi;
  std::string src_ = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)30/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)160, height=(int)120, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  cv::Mat frame_;
  cv::Mat frame_pi;
  size_t width_;
  size_t height_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}