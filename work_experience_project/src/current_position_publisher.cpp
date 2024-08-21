/*
    current_position_publisher.cpp (Current Position Publisher)
    역할: 이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시합니다. 실제 환경에서 로봇의 위치 정보를 얻는 방법에 따라 get_current_position() 함수가 수정될 수 있습니다. 여기서는 예시로 고정된 위치를 퍼블리시하고 있습니다. 이 노드는 "current_position" 토픽에 geometry_msgs::msg::PoseStamped 형식으로 위치 데이터를 보냅니다.
    동작에 필요한 이유: 이 노드는 로봇의 현재 위치를 다른 노드들이 구독하여 사용할 수 있도록 정보를 제공하는 역할을 합니다. 경로 추적이나 목표 위치로의 이동 등에서 현재 위치 정보는 필수적입니다.
    사용 시나리오: 이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시해야 하는 경우에 사용됩니다. 로봇의 위치 정보를 다른 노드들이 필요로 한다면 실행해야 합니다.
    실행 필요성: 현재 위치 정보가 필요하거나 기록되어야 한다면 실행해야 합니다. 예를 들어, 경로 추적이나 위치 기반 작업을 할 때 필요합니다.
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 현재 위치를 주기적으로 퍼블리시하는 노드
class CurrentPositionPublisher : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 지정하고 퍼블리셔와 타이머를 초기화
    CurrentPositionPublisher() : Node("current_position_publisher")
    {
        // 퍼블리셔 초기화: "current_position" 토픽에 PoseStamped 메시지 타입을 퍼블리시
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_position", 10);

        // 타이머 설정: 500ms마다 현재 위치를 퍼블리시
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&CurrentPositionPublisher::publish_current_position, this));
    }

private:
    // 타이머 콜백 함수: 주기적으로 현재 위치를 퍼블리시
    void publish_current_position()
    {
        auto current_position = get_current_position(); // 현재 위치를 가져옴
        publisher_->publish(current_position);          // 퍼블리셔를 통해 퍼블리시
    }

    // 현재 위치를 반환하는 함수 (여기서는 임의의 위치를 설정, 실제 로봇의 위치 데이터를 반환하도록 수정 가능)
    geometry_msgs::msg::PoseStamped get_current_position()
    {
        auto pose = geometry_msgs::msg::PoseStamped(); // PoseStamped 메시지 생성
        pose.header.frame_id = "map";                  // 좌표계 기준을 "map"으로 설정
        pose.header.stamp = this->get_clock()->now();  // 현재 시간을 타임스탬프로 설정
        pose.pose.position.x = 1.0;                    // 임의의 x 좌표
        pose.pose.position.y = 1.0;                    // 임의의 y 좌표
        pose.pose.position.z = 0.0;                    // z 좌표는 사용하지 않음

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);                     // 회전 정보를 쿼터니언으로 설정 (yaw만 사용)
        pose.pose.orientation = tf2::toMsg(q); // 쿼터니언을 메시지로 변환

        return pose; // 생성된 PoseStamped 메시지 반환
    }

    // 퍼블리셔와 타이머 선언
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // CurrentPositionPublisher 노드 실행
    rclcpp::spin(std::make_shared<CurrentPositionPublisher>());
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
