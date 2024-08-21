/*
    waypoints_follower_client.cpp (Waypoints Follower Client)
    역할: 이 노드는 미리 정의된 경로(여러 개의 목표 위치)를 따라가는 액션 클라이언트입니다. 목표 위치는 geometry_msgs::msg::PoseStamped로 정의되며, 이들은 nav2_msgs::action::FollowWaypoints 액션 서버로 전송됩니다. 액션 서버가 목표를 수락하면, 로봇은 경로를 따라 이동합니다. 이 노드는 경로를 전송하고 서버의 응답 및 피드백을 처리합니다.
    동작에 필요한 이유: 이 노드는 로봇이 경로를 따라 이동할 수 있도록 경로 정보를 액션 서버에 전송하는 역할을 합니다. 로봇이 다수의 경유지를 통해 이동해야 하는 경우 필수적인 역할을 수행합니다.
    사용 시나리오: 이 노드는 설정된 경로를 따라 로봇이 이동하도록 할 때 사용됩니다. 주어진 경로에 따라 로봇이 이동해야 하는 경우에 실행됩니다.
    실행 필요성: 경로를 따라 로봇을 이동시키려면 반드시 실행해야 합니다. 이 노드가 실행되지 않으면, 로봇이 설정된 경로를 따라 움직이지 않습니다.
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 로봇이 설정된 경로를 따라가도록 하는 액션 클라이언트 노드
class WaypointsFollower : public rclcpp::Node
{
public:
    // FollowWaypoints 액션 타입 정의
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    // 생성자: 노드 이름을 지정하고 액션 클라이언트를 초기화
    WaypointsFollower(const rclcpp::NodeOptions &options) : Node("waypoints_follower_client", options)
    {
        // 액션 클라이언트 초기화: "follow_waypoints" 액션 서버와 통신
        _action_client = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        // 주기적으로 목표를 전송하기 위한 타이머 초기화
        _timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&WaypointsFollower::send_goal, this));
    }

private:
    // 타이머 콜백 함수: 목표 전송 함수
    void send_goal()
    {
        _timer->cancel(); // 타이머를 취소하여 한 번만 실행되도록 설정

        // 액션 서버가 사용 가능한지 확인
        if (!_action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // 경로를 정의하기 위해 목표 메시지를 생성
        auto goal_msg = FollowWaypoints::Goal();
        // 각 좌표와 각도 (x, y, theta)를 설정하여 경로를 추가
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.44, -0.158, 1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.43, 0.353, 3.14));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.122, 0.465, -1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.0, 0.0, 0.0));

        RCLCPP_INFO(get_logger(), "Sending goal request");

        // 목표 전송 옵션 설정: 응답, 피드백, 결과 콜백 함수 정의
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&WaypointsFollower::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&WaypointsFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&WaypointsFollower::result_callback, this, std::placeholders::_1);

        // 목표를 비동기적으로 전송
        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

    // 좌표와 각도를 사용해 PoseStamped 메시지를 생성하는 함수
    geometry_msgs::msg::PoseStamped get_pose_from_xy_theta(const float &x, const float &y, const float &theta)
    {
        geometry_msgs::msg::PoseStamped pose;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        pose.header.frame_id = "map";                 // 좌표계 기준을 "map"으로 설정
        pose.header.stamp = this->get_clock()->now(); // 현재 시간을 타임스탬프로 설정
        pose.pose.position.x = x;                     // x 좌표 설정
        pose.pose.position.y = y;                     // y 좌표 설정
        pose.pose.position.z = 0.0;                   // z 좌표는 사용하지 않음
        pose.pose.orientation = tf2::toMsg(q);        // 쿼터니언을 메시지로 변환하여 설정

        return pose;
    }

    // 목표 응답 콜백 함수: 목표가 수락되었는지 여부를 확인
    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // 피드백 콜백 함수: 현재 경유지 정보를 출력
    void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                           const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", feedback->current_waypoint);
    }

    // 결과 콜백 함수: 목표 달성 결과를 처리
    void result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!");
            return;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_INFO(get_logger(), "Unknown goal result code");
            return;
        }
    }

    // 액션 클라이언트와 타이머 선언
    rclcpp_action::Client<FollowWaypoints>::SharedPtr _action_client;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // WaypointsFollower 노드 실행
    auto node = std::make_shared<WaypointsFollower>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
