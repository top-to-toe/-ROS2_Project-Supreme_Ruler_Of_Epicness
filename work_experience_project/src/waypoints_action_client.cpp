#include "rclcpp/rclcpp.hpp"                       // ROS2 C++ API
#include "geometry_msgs/msg/pose_stamped.hpp"      // 위치 메시지 타입
#include "nav2_msgs/action/follow_waypoints.hpp"   // 경유지 따라가기 액션 메시지
#include "rclcpp_action/rclcpp_action.hpp"         // 액션 클라이언트 관련 API
#include "tf2/LinearMath/Quaternion.h"             // 쿼터니언을 위한 TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // TF2 메시지 변환

class WaypointsActionClient : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;                         // 액션 타입 정의
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>; // 액션 핸들 정의

    WaypointsActionClient(const rclcpp::NodeOptions &options)
        : Node("waypoints_action_client", options)
    {
        // 액션 클라이언트 생성
        this->client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");

        // 타이머를 이용해 경유지 목표를 전송하는 콜백을 주기적으로 호출
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&WaypointsActionClient::send_waypoints_goal, this));
    }

private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client_; // 액션 클라이언트
    rclcpp::TimerBase::SharedPtr timer_;                       // 타이머

    // 경유지 목표를 설정하고 전송하는 함수
    void send_waypoints_goal()
    {
        // 액션 서버가 준비될 때까지 대기
        if (!this->client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowWaypoints::Goal(); // 경유지 목표 메시지 생성

        // 각 포인트에 대한 위치 및 회전 정보 설정
        goal_msg.poses.push_back(create_pose(-0.295, -0.506, 0.0)); // 1번째 포인트
        goal_msg.poses.push_back(create_pose(-0.657, -0.316, 0.0)); // 2번째 포인트
        goal_msg.poses.push_back(create_pose(-0.748, 0.0933, 0.0)); // 3번째 포인트
        goal_msg.poses.push_back(create_pose(0.0, 0.0, 0.0));       // 4번째 포인트

        // 목표 전송 옵션 설정
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointsActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointsActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointsActionClient::result_callback, this, std::placeholders::_1);

        // 비동기적으로 목표 전송
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

    // x, y 좌표와 theta 값을 이용해 PoseStamped 메시지 생성
    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double theta)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";    // 좌표계는 "map"
        pose.header.stamp = this->now(); // 현재 시간을 타임스탬프로 설정

        pose.pose.position.x = x;   // x 위치 설정
        pose.pose.position.y = y;   // y 위치 설정
        pose.pose.position.z = 0.0; // z 위치는 0으로 설정 (2D 평면)

        tf2::Quaternion q;                     // 쿼터니언 생성
        q.setRPY(0, 0, theta);                 // 회전을 theta 값으로 설정
        pose.pose.orientation = tf2::toMsg(q); // 쿼터니언을 메시지 형식으로 변환

        return pose; // PoseStamped 메시지 반환
    }

    // 목표 수락/거절에 대한 콜백 함수
    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // 경유지 이동 중에 실시간 피드백을 받는 콜백 함수
    void feedback_callback(
        GoalHandleFollowWaypoints::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Currently at waypoint %u", feedback->current_waypoint);
    }

    // 목표 완료 후 결과를 받는 콜백 함수
    void result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            return;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                                   // ROS2 초기화
    auto node = std::make_shared<WaypointsActionClient>(rclcpp::NodeOptions()); // 노드 생성
    rclcpp::spin(node);                                                         // 노드 실행
    rclcpp::shutdown();                                                         // ROS2 종료
    return 0;
}
