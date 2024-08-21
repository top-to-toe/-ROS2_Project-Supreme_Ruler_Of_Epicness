/*
    waypoints_service_server.cpp (Waypoints Service Server)
    역할: 이 노드는 경로 점(waypoints)을 설정하는 서비스 서버입니다. 클라이언트로부터 x, y, theta 배열로 정의된 경로 점을 받아 이를 내부적으로 저장합니다. 이러한 경로 점들은 로봇이 따라갈 경로를 형성합니다.
    동작에 필요한 이유: 이 노드는 로봇의 경로를 유연하게 설정할 수 있도록 합니다. 클라이언트가 서비스 호출을 통해 원하는 경로를 정의하고 설정할 수 있으며, 다른 노드들이 이 경로 정보를 이용해 로봇을 제어할 수 있습니다.
    사용 시나리오: 이 노드는 클라이언트로부터 경로 점(waypoints)을 설정할 때 사용됩니다. 따라서 경로를 동적으로 설정하고자 할 때 반드시 실행해야 합니다.
    실행 필요성: 경로 점을 설정하는 데 필요합니다. 이 노드가 실행되지 않으면, 클라이언트는 경로 점을 설정할 수 없습니다.
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "work_experience_project/srv/waypoint_service.hpp" // WaypointService 서비스 타입을 사용하기 위한 헤더 파일
#include "tf2/LinearMath/Quaternion.h"
#include <memory>
#include <vector>

// 경로점을 설정하는 서비스 서버 노드
class WaypointsService : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 지정하고 서비스 서버를 초기화
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
        // 요청으로부터 받은 x, y, theta 배열의 크기가 일치하는지 확인
        if (request->x.size() == request->y.size() && request->x.size() == request->theta.size())
        {
            waypoints_.clear(); // 기존 경로점을 초기화
            // 받은 모든 좌표와 각도(x, y, theta)를 경로점으로 변환하여 리스트에 추가
            for (size_t i = 0; i < request->x.size(); ++i)
            {
                waypoints_.emplace_back(get_pose_from_xy_theta(request->x[i], request->y[i], request->theta[i]));
            }
            response->success = true;
            response->message = "Waypoints set successfully."; // 성공 메시지
            RCLCPP_INFO(this->get_logger(), "Received %zu waypoints.", request->x.size());
        }
        else
        {
            response->success = false;
            response->message = "Array sizes for x, y, theta do not match."; // 실패 메시지
            RCLCPP_ERROR(this->get_logger(), "Mismatched array sizes for x, y, theta.");
        }
    }

    // x, y, theta를 기반으로 PoseStamped 메시지를 생성하는 함수
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

    // 경로점을 저장하는 벡터와 서비스 서버 선언
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    rclcpp::Service<work_experience_project::srv::WaypointService>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // WaypointsService 노드 실행
    auto node = std::make_shared<WaypointsService>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
