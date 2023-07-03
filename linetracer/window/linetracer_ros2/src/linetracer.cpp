#include <functional> //헤더파일 포함
#include <memory> //헤더파일 포함
#include <cstdio> //헤더파일 포함
#include <iostream> //헤더파일 포함
#include <string> //헤더파일 포함
#include "opencv2/opencv.hpp" //헤더파일 포함
#include "rclcpp/rclcpp.hpp" //헤더파일 포함
#include "sensor_msgs/msg/image.hpp" //헤더파일 포함
#include "linetracer_ros2/camera.hpp" //헤더파일 포함
#include <chrono> //헤더파일 포함
#include "geometry_msgs/msg/twist.hpp" //헤더파일 포함
#include <conio.h> //헤더파일 포함
using namespace std; //std:: 를 생략
using namespace cv; //cv:: 를 생략
using namespace std::chrono_literals; //std::chrono_literals 를 생략
using std::placeholders::_1; //첫번째 인수를 나타내는 플레이스홀더 

class LineTracerW : public rclcpp::Node //LineTracerW 메인 클래스로 rclcpp의 node 클래스 상속 
{
  public:
  LineTracerW() : Node("Camera_subscriber") //클래스 생성자 정의
  {
    size_t depth = rmw_qos_profile_default.depth; //depth를 디폴트 값으로 설정
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability; //QoS 설정에서 신뢰성을 기본값으로 설정
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history; //QoS설정에서 메시지의 히스토리정책을 가장 기본적인 히스토리를 사용
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth)); //rclcpp 라이브러리를 사용해 QoS의 프로파일 생성
    reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT; //가장 높은 성능과 낮은 신뢰성인 설정을 policy에 저장
    qos_profile.reliability(reliability_policy);  //qos_profile의 신뢰성을 reliability_policy의 신뢰성으로 변경

    //카메라 영상을 subscriber
    camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image", qos_profile,
                                    std::bind(&LineTracerW::show_image, this, _1));

    //dynamixel QoS profile 생성과 다이나믹셀 publisher
    auto dynamix_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", dynamix_qos_profile);
    timer_ = this->create_wall_timer(10ms, std::bind(&LineTracerW::publish_velcmd_msg, this));
    
    //창 생성
    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
  }
 private:
    void show_image(const sensor_msgs::msg::Image::SharedPtr msg) //이미지 출력 함수
    {
        RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str()); //로그 메시지
        
        // 수신한 이미지 메시지를 opencv의 mat형식으로 변환
        Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
        const_cast<unsigned char*>(msg->data.data()), msg->step);
        //rgb형태이면 bgr로 변환
        if (msg->encoding == "rgb8") {
            cv::cvtColor(frame, frame, COLOR_RGB2BGR);
        }

        Mat cvframe = frame;
        Mat gray, dst;

        cvtColor(cvframe,gray,COLOR_BGR2GRAY); //bgr -> gray로 변환

        threshold(gray, gray, 200, 255, THRESH_BINARY); //이진화

        dst=gray(Rect(0,3*gray.rows/4,gray.cols,gray.rows/4)); //관심영역 추출
        
        Mat labels,status,centroids;
        int cnt=connectedComponentsWithStats(dst,labels,status,centroids); //검출된 객체의 정보를 가져옴

        int count=0;
        int max_size=0;
        for(int i=1;i<cnt;i++) //검출된 객체 중에 면적이 가장 큰 객체를 찾는 함수
        {
            int *l=status.ptr<int>(i);
            if(l[4]>max_size)
            {
                max_size=l[4];
                count=i;
            }
        }
        int *max_l=status.ptr<int>(count); //면적이 가장큰 객체의 정보
        double *lc=centroids.ptr<double>(count); //면적이 가장큰 객체의 중심좌표
        cvtColor(dst,dst,COLOR_GRAY2BGR); //gray-> bgr로 변환
        circle(dst,Point2d(lc[0],lc[1]),3,Scalar(0,0,255),-1); //중심에 원을 그려줌
        rectangle(dst,Rect(max_l[0],max_l[1],max_l[2],max_l[3]),Scalar(0,0,255),1);//찾은 객체의 사각형을 그려줌
        error=(gray.cols/2-lc[0]); //화면의 중심에서 찾은 객체의 중심의 차를 error로 이용
        
        if(abs(old_error-error)>20)error=old_error; //error 이전 값이 error 이후 값이랑 20차이가 나면 이전 error를 사용 

        imshow("showimage", cvframe); //기본 이미지 출력
        imshow("dst", dst); //이진화한 이미지 출력
        waitKey(1);
    }
    void publish_velcmd_msg() //메세지를 전송할 함수
    {
        static auto msg = geometry_msgs::msg::Twist(); 
        msg.linear.x = 0.3; //선속도
        msg.angular.z = error/70; //각속도
        RCLCPP_INFO(this->get_logger(), "Published message: %lf,%lf, error : %lf",msg.linear.x, msg.linear.y, msg.angular.z); //로그 메시지
        dynamixel_publisher_->publish(msg); //메시지 발행  
        old_error=error; //이전 error값에 현재 error값 대입
    }
//변수 선언
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_;
double error;
double old_error=0.0;
};

int main(int argc, char * argv[]) //메인 함수 
{
  rclcpp::init(argc, argv); //초기화
  auto node = std::make_shared<LineTracerW>(); //LineTracerW 클래스의 node 생성
  rclcpp::spin(node); //실행루프 시작해 노드를 실행
  rclcpp::shutdown(); //ROS 2의 실행을 종료
  return 0; //반환값 0
}
