#include <cstdio> //헤더파일 포함
#include <iostream> //헤더파일 포함
#include <memory> //헤더파일 포함
#include <string> //헤더파일 포함
#include <utility> //헤더파일 포함
#include <sstream> //헤더파일 포함
#include "opencv2/opencv.hpp" //헤더파일 포함
#include "rclcpp/rclcpp.hpp" //헤더파일 포함
#include "sensor_msgs/msg/image.hpp" //헤더파일 포함
#include "linetracer_ros2/camera.hpp" //헤더파일 포함
#include <functional> //헤더파일 포함
#include "geometry_msgs/msg/twist.hpp" //헤더파일 포함
#include "linetracer_ros2/dxl.hpp" //헤더파일 포함
using namespace std; //std:: 를 생략
using namespace cv; //cv:: 를 생략
using namespace std::chrono_literals; //std::chrono_literals 를 생략
using std::placeholders::_1; //첫번째 인수를 나타내는 플레이스홀더

class LineTracerL : public rclcpp::Node //LineTracerL 메인 클래스로 rclcpp의 node 클래스 상속
{
  public:
  LineTracerL() : Node("linetracer"), count_(1), width_(320), height_(240) //클래스 생성자 정의 및 초기화
  {
    //QoS 설정
    size_t depth = rmw_qos_profile_default.depth; //depth를 디폴트 값으로 설정
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; //QoS 설정에서 신뢰성을 가장 높은 성능과 낮은 신뢰성으로 설정 
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; //QoS설정에서 메시지의 히스토리정책을 가장 기본적인 히스토리를 사용
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth)); //rclcpp 라이브러리를 사용해 QoS의 프로파일 생성
    qos_profile.reliability(reliability_policy); //qos_profile의 신뢰성을 reliability_policy의 신뢰성으로 변경

    //카메라 영상을 publisher_
    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos_profile);
    timer_ = this->create_wall_timer(30ms, std::bind(&LineTracerL::publish_image, this));
    
    //dynamixel QoS profile 생성과 다이나믹셀 subscriber_
    auto qos_profile_dxl = rclcpp::QoS(rclcpp::KeepLast(10));
    dynamixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos_profile_dxl, std::bind(&LineTracerL::subscribe_topic_message, this, _1));

    cap_.open(src_, cv::CAP_GSTREAMER); //cam을 gstreamer로 열기
    
    if (!cap_.isOpened()) //카메라가 열리지 않는다면
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream"); //에러메시지 출력    
    }
    if(!dxl.dxl_open()) RCLCPP_ERROR(this->get_logger(), "dynamixel open error"); //다이나믹셀이 열리지 않는다면 메시지 출력
  }
  ~LineTracerL() //소멸자
  {
    dxl.dxl_close(); //dxl종료
  }
  private:
  void publish_image() //이미지를 불러오기
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>(); //객체의 메모리 할당 및 해체 자동 처리
    msg->is_bigendian = false; //인수 지정
    cap_ >> frame_; //영상을 받아옴
    if (!frame_.empty())  //영상이 없다면 
    {
      // Convert to a ROS image
      // cv::flip(frame, frame, 1); // Flip the frame if needed
      //convert_frame_to_message(frame_, count_, *msg);
      convert_frame_to_message(frame_, *msg); //프레임을 메시지로 변환
      
      //cv::Mat cvframe = frame_;
      //cv::imshow("camimage", cvframe);
      //cv::waitKey(1);
      
      // Publish the image message and increment the frame_id.
      RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", count_++); //로그 메시지
      camera_publisher_->publish(std::move(msg)); //메시지 형태로 발행      
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "frame empty"); //로그 메시지
    }
  }
  void subscribe_topic_message(const geometry_msgs::msg::Twist::SharedPtr msg) //토픽 메시지를 subscriber 하는 함수
  {
    RCLCPP_INFO(this->get_logger(), "Received message: %lf,%lf", msg->linear.x,msg->angular.z); //로그 메시지
    int r_rpm1=0, r_rpm2=0; //변수 초기화
    dxl.convert_LARPM(msg->linear.x,msg->angular.z,&r_rpm1,&r_rpm2); //각속도 선속도를 rpm으로 변환해주는 함수
    std::cout<<"r_rpm1:"<<r_rpm1<<std::endl; //메시지 출력
    std::cout<<"r_rpm2:"<<-r_rpm2<<std::endl; //메시지 출력
    bool result = dxl.dxl_set_velocity(r_rpm1, -r_rpm2); //다이나믹셀 동작
    if(result == false) //result가 false이면 
    {
        RCLCPP_ERROR(this->get_logger(), "dxl_set_velocity error"); //에러 로그메시지 출력 
    }
  }
  //변수 선언
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dynamixel_subscriber_;
  Dxl dxl;
  size_t count_;
  cv::VideoCapture cap_;
  std::string src_ = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)320, height=(int)240, format=(string)NV12, framerate=(fraction)30/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)320, height=(int)240, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  cv::Mat frame_;
  size_t width_;
  size_t height_;
};

int main(int argc, char * argv[]) //메인 함수
{
  rclcpp::init(argc, argv); //초기화
  auto node = std::make_shared<LineTracerL>(); //LineTracerL 클래스의 node 생성
  rclcpp::spin(node); //실행루프 시작해 노드를 실행
  rclcpp::shutdown(); //ROS 2의 실행을 종료
  return 0; //반환값 0
}
