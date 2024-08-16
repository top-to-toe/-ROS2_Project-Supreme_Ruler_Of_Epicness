#include "rclcpp/rclcpp.hpp"                    // ROS 2 C++ 클라이언트를 위한 기본 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp"   // 위치와 자세를 나타내는 메시지 타입
#include "sensor_msgs/msg/compressed_image.hpp" // 카메라 이미지 데이터를 위한 메시지 정의
#include "std_srvs/srv/set_bool.hpp"            // LED를 제어하기 위한 서비스 메시지 정의
#include "std_msgs/msg/string.hpp"              // 문자열 메시지 타입
#include <chrono>                               // 시간 관련 기능을 위한 헤더 파일

using namespace std::chrono_literals; // 시간 리터럴 사용을 위한 네임스페이스

class MainControlNode : public rclcpp::Node
{
public:
    MainControlNode() : Node("main_control_node") // 노드 이름을 "main_control_node"로 설정
    {
        // 위치 정보를 퍼블리시하는 퍼블리셔 생성
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic", 10);

        // LED를 제어하는 서비스 클라이언트 생성
        led_client_ = this->create_client<std_srvs::srv::SetBool>("toggle_led");

        // 좌표 정보를 수신하는 서브스크립션 생성
        coordinate_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "coordinate_topic", 10, std::bind(&MainControlNode::coordinate_callback, this, std::placeholders::_1));

        // 카메라 이미지를 수신하는 서브스크립션 생성
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", 10, std::bind(&MainControlNode::image_callback, this, std::placeholders::_1));
    }

private:
    // 좌표 정보를 수신할 때 호출되는 콜백 함수
    void coordinate_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream iss(msg->data);
        float x, y;
        iss >> x >> y;

        // 수신된 좌표 정보 로그 출력
        RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%f, y=%f", x, y);

        // 위치 정보를 담은 메시지 생성
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        // 위치 정보를 퍼블리시
        pose_publisher_->publish(pose_msg);

        // LED를 켜기 위한 서비스 요청 메시지 생성
        auto led_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        led_request->data = true;

        // LED 서비스가 사용 가능해질 때까지 대기
        while (!led_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for LED service...");
        }

        // LED 제어 서비스 요청 전송
        auto result = led_client_->async_send_request(led_request);

        try
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "LED Service response: %s", response->message.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    // 카메라 이미지 수신 시 호출되는 콜백 함수
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // 이미지의 타임스탬프 정보를 로그로 출력
        RCLCPP_INFO(this->get_logger(), "Received image with timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;        // 위치 정보를 퍼블리시하는 퍼블리셔
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr led_client_;                        // LED 제어를 위한 서비스 클라이언트
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr coordinate_subscriber_;        // 좌표 정보 수신을 위한 서브스크립션
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_; // 카메라 이미지 수신을 위한 서브스크립션
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                          // ROS 2 초기화
    rclcpp::spin(std::make_shared<MainControlNode>()); // 노드 실행 및 스핀
    rclcpp::shutdown();                                // ROS 2 종료
    return 0;
}
