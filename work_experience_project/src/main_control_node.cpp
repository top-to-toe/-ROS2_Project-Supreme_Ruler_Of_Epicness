/*
    이 노드는 로봇의 상태를 관리하고, 카메라 피드를 실시간으로 표시하며, LED를 수동으로 제어할 수 있는 기능을 제공합니다.
    또한 경로의 목표 지점을 설정하고, 목표 수행 결과를 처리합니다.
    LED 제어와 이동 명령 전송은 각각 독립적인 스레드에서 비동기적으로 처리됩니다.
*/

#include "rclcpp/rclcpp.hpp"                    // ROS 2의 rclcpp 라이브러리를 사용하기 위한 헤더 파일
#include "std_msgs/msg/string.hpp"              // 문자열 메시지 타입을 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp"   // PoseStamped 메시지 타입을 사용하기 위한 헤더 파일
#include "sensor_msgs/msg/compressed_image.hpp" // 압축된 이미지 메시지 타입을 사용하기 위한 헤더 파일
#include "cv_bridge/cv_bridge.h"                // ROS 이미지 메시지를 OpenCV 이미지로 변환하기 위한 헤더 파일
#include "opencv2/opencv.hpp"                   // OpenCV의 주요 기능을 사용하기 위한 헤더 파일
#include "std_srvs/srv/set_bool.hpp"            // SetBool 서비스 타입을 사용하기 위한 헤더 파일
#include "std_srvs/srv/trigger.hpp"             // Trigger 서비스 타입을 사용하기 위한 헤더 파일
#include <thread>
#include <future>

using namespace std::chrono_literals; // 500ms와 같은 리터럴을 사용하기 위한 네임스페이스

class MainControlNode : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 설정하고 서비스 클라이언트, 퍼블리셔, 서브스크립션을 초기화합니다.
    MainControlNode() : Node("main_control_node")
    {
        using namespace std::placeholders;

        // 로봇의 현재 위치 상태를 구독하는 subscription 객체 생성
        state_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "robot_state", 10, std::bind(&MainControlNode::state_callback, this, _1));

        // waypoints_action_client로 새로운 목표 경유지를 퍼블리시하기 위한 publisher 객체 생성
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint_goal", 10);

        // 카메라 이미지 데이터를 구독하기 위한 subscription 객체 생성
        image_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", 10, std::bind(&MainControlNode::image_callback, this, _1));

        // LED 제어를 위한 서비스 클라이언트 생성
        led_client_ = this->create_client<std_srvs::srv::SetBool>("toggle_led");

        // 목표 전송을 위한 서비스 클라이언트 생성
        goal_service_client_ = this->create_client<std_srvs::srv::Trigger>("execute_waypoint_goal");

        // 스레드에서 서비스 호출을 처리하기 위한 람다 함수
        // service_thread_ = std::thread([this]()
        //                               {
        //     rclcpp::Rate loop_rate(10);
        //     while (rclcpp::ok()) {
        //         rclcpp::spin_some(this->get_node_base_interface());
        //         loop_rate.sleep();
        //     } });
    }

    ~MainControlNode()
    {
        if (service_thread_.joinable())
            service_thread_.join();
    }

    // 수동으로 LED를 켜거나 끄는 함수
    void toggle_led(bool turn_on)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = turn_on; // LED 상태 설정 (true: 켜기, false: 끄기)

        // 서비스 요청을 비동기적으로 실행
        auto future = led_client_->async_send_request(request);
        handle_service_response<std_srvs::srv::SetBool>(future.share(), "LED control");
    }

    // Waypoint 목표를 서비스 서버에 전송하는 함수
    void send_waypoint_goal()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = goal_service_client_->async_send_request(request);
        handle_service_response<std_srvs::srv::Trigger>(future.share(), "Waypoint execution");
    }

private:
    // 로봇의 현재 상태를 구독하는 콜백 함수
    void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 로봇의 현재 위치를 로그로 출력
        RCLCPP_INFO(this->get_logger(), "Current robot position - x: %f, y: %f", msg->pose.position.x, msg->pose.position.y);
    }

    // 카메라 이미지 콜백 함수
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // 압축된 이미지 메시지를 OpenCV 이미지로 변환
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image; // 변환된 이미지를 OpenCV Mat 형식으로 저장
            // 이미지를 윈도우에 표시
            cv::imshow("Camera Feed", img);
            cv::waitKey(10); // 화면 갱신 주기 설정 (10ms)
        }
        catch (cv_bridge::Exception &e)
        {
            // 이미지 변환 실패 시 에러 로그 출력
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
        }
    }

    // 서비스 호출 결과를 처리하는 함수
    template <typename T>
    void handle_service_response(std::shared_future<typename T::Response::SharedPtr> future, const std::string &service_name)
    {
        std::thread([future, service_name, this]()
                    {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "%s succeeded: %s", service_name.c_str(), response->message.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "%s failed: %s", service_name.c_str(), response->message.c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call to %s failed: %s", service_name.c_str(), e.what());
            } })
            .detach(); // 스레드를 detach하여 독립적으로 실행되도록 함
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr led_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr goal_service_client_;
    std::thread service_thread_;
};

int main(int argc, char *argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainControlNode>();

    // 예제: 수동으로 LED를 켜거나 끄는 로직 추가
    node->toggle_led(true);                               // LED 켜기
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 2초 대기
    node->send_waypoint_goal();                           // 경로 실행 요청
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 2초 대기
    node->toggle_led(false);                              // LED 끄기

    // 노드를 스핀하여 실행
    rclcpp::spin(node);
    // ROS 2 종료
    rclcpp::shutdown();
    return 0;
}

/*
    MainControlNode 클래스는 로봇의 상태를 관리하고, 카메라 피드를 처리하며, 수동으로 LED를 제어하는 기능을 제공합니다.
    toggle_led 함수는 LED를 켜거나 끄는 기능을 수행하며, 비동기적으로 std_srvs::srv::SetBool 서비스를 호출합니다.
    send_waypoint_goal 함수는 로봇의 경로 목표를 설정하고, 이를 비동기적으로 처리합니다.
    state_callback 및 image_callback 함수는 로봇의 상태 및 카메라 이미지를 처리합니다.
    service_thread_는 비동기 작업을 처리하기 위해 별도의 스레드를 사용합니다.

    이 노드는 로봇의 상태를 관리하고, 카메라 피드를 실시간으로 표시하며, LED를 수동으로 제어할 수 있는 기능을 제공합니다.
    또한 경로의 목표 지점을 설정하고, 목표 수행 결과를 처리합니다.
*/