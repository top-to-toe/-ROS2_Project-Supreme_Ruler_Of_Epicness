#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "work_experience_project/srv/waypoint_service.hpp" // WaypointService 서비스 타입을 사용하기 위한 헤더 파일
#include "tf2/LinearMath/Quaternion.h"
#include <memory>
#include <vector>

class WaypointsService : public rclcpp::Node
{
public:
    WaypointsService(const rclcpp::NodeOptions &options) : Node("waypoints_service_server", options)
    {
        // 서비스 서버 초기화: "send_waypoint" 서비스 요청을 처리
        service_ = this->create_service<work_experience_project::srv::WaypointService>(
            "send_waypoint",
            std::bind(&WaypointsService::handle_waypoint_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 서비스 요청을 처리하는 함수
    void handle_waypoint_request(
        const std::shared_ptr<work_experience_project::srv::WaypointService::Request> request,
        std::shared_ptr<work_experience_project::srv::WaypointService::Response> response)
    {
        if (request->x.size() == request->y.size() && request->x.size() == request->theta.size())
        {
            waypoints_.clear();
            for (size_t i = 0; i < request->x.size(); ++i)
            {
                waypoints_.emplace_back(get_pose_from_xy_theta(request->x[i], request->y[i], request->theta[i]));
            }
            response->success = true;
            response->message = "Waypoints set successfully.";
            RCLCPP_INFO(this->get_logger(), "Received %zu waypoints.", request->x.size());
        }
        else
        {
            response->success = false;
            response->message = "Array sizes for x, y, theta do not match.";
            RCLCPP_ERROR(this->get_logger(), "Mismatched array sizes for x, y, theta.");
        }
    }

    // x, y, theta를 기반으로 PoseStamped 메시지를 생성하는 함수 (모태 코드 그대로 유지)
    geometry_msgs::msg::PoseStamped get_pose_from_xy_theta(const float &x, const float &y, const float &theta)
    {
        auto pose = geometry_msgs::msg::PoseStamped(); // PoseStamped 메시지 생성
        tf2::Quaternion q;                             // 회전을 나타내는 쿼터니언 생성
        q.setRPY(0, 0, theta);                         // yaw(세타) 값만을 사용하여 쿼터니언 계산

        pose.header.frame_id = "map";                          // Pose의 기준 좌표계를 "map"으로 설정
        pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(); // 현재 시간을 타임스탬프로 설정
        pose.pose.position.x = x;                              // x 좌표 설정
        pose.pose.position.y = y;                              // y 좌표 설정
        pose.pose.position.z = 0.0;                            // z 좌표는 0으로 설정
        pose.pose.orientation.x = q.x();                       // 쿼터니언의 x값 설정
        pose.pose.orientation.y = q.y();                       // 쿼터니언의 y값 설정
        pose.pose.orientation.z = q.z();                       // 쿼터니언의 z값 설정
        pose.pose.orientation.w = q.w();                       // 쿼터니언의 w값 설정
        return pose;                                           // 생성된 PoseStamped 메시지 반환
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    rclcpp::Service<work_experience_project::srv::WaypointService>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsService>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
