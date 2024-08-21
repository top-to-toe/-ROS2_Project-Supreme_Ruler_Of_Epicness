#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // getYaw 함수를 사용하기 위한 헤더 파일

class TargetPositionSubscriber : public rclcpp::Node
{
public:
    TargetPositionSubscriber() : Node("target_position_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_position", 10, std::bind(&TargetPositionSubscriber::target_position_callback, this, std::placeholders::_1));
    }

private:
    void target_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // tf2::Quaternion 객체를 사용해 쿼터니언에서 yaw 값을 추출
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // roll, pitch, yaw 값 추출

        RCLCPP_INFO(this->get_logger(), "Received target position: x=%.2f, y=%.2f, theta=%.2f",
                    msg->pose.position.x, msg->pose.position.y, yaw);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPositionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
