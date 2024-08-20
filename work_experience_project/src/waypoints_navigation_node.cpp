/*
    이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시하여 다른 노드들이 로봇의 상태를 알 수 있도록 합니다.
    위치 정보는 `geometry_msgs::msg::PoseStamped` 메시지 타입으로 퍼블리시되며, 위치 정보는 "robot_state" 토픽을 통해 배포됩니다.
*/

#include "rclcpp/rclcpp.hpp"                  // ROS 2의 rclcpp 라이브러리를 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp" // PoseStamped 메시지 타입을 사용하기 위한 헤더 파일
#include <chrono>                             // 시간 관련 기능을 사용하기 위한 표준 라이브러리

using namespace std::chrono_literals; // 500ms와 같은 리터럴을 사용하기 위한 네임스페이스

class WaypointsNavigationNode : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 설정하고 퍼블리셔와 타이머를 초기화합니다.
    WaypointsNavigationNode() : Node("waypoints_navigation_node") // 노드 이름을 "waypoints_navigation_node"로 설정하여 생성자 정의
    {
        // 로봇의 현재 위치를 퍼블리시하기 위한 퍼블리셔 객체 생성
        state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_state", 10);

        // 타이머를 설정하여 주기적으로 로봇의 상태를 퍼블리시
        timer_ = this->create_wall_timer(
            500ms, std::bind(&WaypointsNavigationNode::publish_robot_state, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_; // 로봇 상태 퍼블리시를 위한 퍼블리셔 객체
    rclcpp::TimerBase::SharedPtr timer_;                                            // 주기적으로 작업을 수행하기 위한 타이머 객체

    // 로봇의 현재 상태를 퍼블리시하는 함수
    void publish_robot_state()
    {
        auto pose = geometry_msgs::msg::PoseStamped(); // PoseStamped 메시지 생성

        // ★★★★★ 실제 좌표를 설정해야 합니다. 예시로 x=1.0, y=1.0으로 설정했습니다.
        pose.pose.position.x = 1.0;    // 임의의 x 좌표 설정
        pose.pose.position.y = 1.0;    // 임의의 y 좌표 설정
        pose.pose.orientation.w = 1.0; // 임의의 회전값 설정

        pose.header.stamp = this->now(); // 현재 시간을 헤더에 설정
        pose.header.frame_id = "map";    // 좌표계를 "map"으로 설정

        state_publisher_->publish(pose); // 생성된 PoseStamped 메시지를 퍼블리시
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                  // ROS 2 초기화
    rclcpp::spin(std::make_shared<WaypointsNavigationNode>()); // WaypointsNavigationNode 노드를 실행하여 콜백을 처리
    rclcpp::shutdown();                                        // ROS 2 종료
    return 0;
}

/*
    이 노드는 로봇의 현재 위치를 지속적으로 퍼블리시합니다.
    geometry_msgs::msg::PoseStamped 메시지를 "robot_state"라는 토픽으로 발행합니다. 이 메시지는 로봇의 현재 좌표(x, y)와 회전(orientation)을 나타냅니다.
    다른 노드들은 이 토픽을 구독하여 로봇의 상태를 추적할 수 있습니다.

    이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시하는 역할을 합니다.
    위치 정보는 "robot_state"라는 토픽을 통해 다른 노드로 전달됩니다.
*/