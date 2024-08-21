#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class CurrentPositionPublisher : public rclcpp::Node
{
public:
    CurrentPositionPublisher() : Node("current_position_publisher")
    {
        // 퍼블리셔 초기화: "current_position" 토픽에 현재 위치를 퍼블리시
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_position", 10);

        // 타이머 설정: 500ms마다 현재 위치를 퍼블리시
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&CurrentPositionPublisher::publish_current_position, this));
    }

private:
    // 현재 위치를 퍼블리시하는 함수
    void publish_current_position()
    {
        auto current_position = get_current_position();
        publisher_->publish(current_position);
    }

    // 현재 위치를 반환하는 함수 (실제 로봇의 위치를 반환하는 로직으로 대체 가능)
    geometry_msgs::msg::PoseStamped get_current_position()
    {
        auto pose = geometry_msgs::msg::PoseStamped(); // PoseStamped 메시지 생성
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = 1.0; // 예시 위치
        pose.pose.position.y = 1.0; // 예시 위치
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // 예시 회전 (yaw만 사용)
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
