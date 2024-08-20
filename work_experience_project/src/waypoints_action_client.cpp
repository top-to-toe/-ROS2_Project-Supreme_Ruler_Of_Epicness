/*
    이 노드는 로봇이 설정된 경유지로 이동하도록 명령을 보내고, 이동이 시작될 때 LED를 켜고 이동이 완료되면 LED를 끄는 기능을 제공합니다.
    이 노드에서는 경유지 좌표를 설정하고, 로봇을 해당 좌표로 이동시키는 작업을 수행합니다.
    또한, 이동 도중 발생하는 피드백을 처리하고, 이동이 완료된 후 후속 작업을 수행할 수 있습니다.
*/

#include "geometry_msgs/msg/pose_stamped.hpp"    // 경유지의 좌표 정보를 담는 메시지 타입
#include "nav2_msgs/action/navigate_to_pose.hpp" // 로봇을 특정 위치로 이동시키기 위한 액션 메시지 타입
#include "rclcpp/rclcpp.hpp"                     // ROS 2의 기본 클라이언트 라이브러리
#include "rclcpp_action/rclcpp_action.hpp"       // ROS 2 액션 클라이언트를 사용하기 위한 라이브러리
#include "std_srvs/srv/set_bool.hpp"             // LED 제어를 위한 서비스 메시지 타입
#include "tf2/LinearMath/Quaternion.h"           // tf2 쿼터니언 라이브러리
#include <thread>                                // 스레드를 사용하기 위한 표준 라이브러리
#include <future>                                // 비동기 작업을 위한 표준 라이브러리

using namespace std::chrono_literals; // 시간 리터럴(예: 10s)을 사용하기 위한 네임스페이스

class WaypointsServiceClient : public rclcpp::Node
{
public:
    // 액션 및 서비스 클라이언트를 위한 타입 정의
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // 생성자: 노드 이름을 설정하고 클라이언트를 초기화합니다.
    WaypointsServiceClient(const rclcpp::NodeOptions &options) : Node("waypoints_service_client", options)
    {
        // navigate_to_pose 액션 클라이언트 생성
        navigate_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // LED 제어를 위한 서비스 클라이언트 생성
        led_client_ = this->create_client<std_srvs::srv::SetBool>("toggle_led");

        // ROS 2 노드가 실행되는 동안 서비스 요청을 처리하기 위한 스레드 생성
        service_thread_ = std::thread([this]()
                                      {
            rclcpp::Rate loop_rate(10); // 루프 주기를 10Hz로 설정
            while (rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface()); // 서비스 요청 처리
                loop_rate.sleep();
            } });
    }

    // 소멸자: 서비스 스레드를 안전하게 종료합니다.
    ~WaypointsServiceClient()
    {
        if (service_thread_.joinable())
        {
            service_thread_.join();
        }
    }

    // 경유지로 이동하는 함수
    void move_to_waypoints()
    {
        // ★★★★★ 제공된 경유지 좌표 및 원점을 추가했습니다.
        std::vector<geometry_msgs::msg::PoseStamped> waypoints = {
            create_pose(1.44, -0.158, 0.0), // 첫 번째 경유지
            create_pose(1.43, 0.353, 0.0),  // 두 번째 경유지
            create_pose(0.122, 0.465, 0.0), // 세 번째 경유지
            create_pose(0.0, 0.0, 0.0)      // 원점 (마지막 도착지)
        };

        for (const auto &waypoint : waypoints)
        {
            // 각 경유지로 이동을 시도합니다.
            if (navigate_to_pose(waypoint))
            {
                RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint at x: %f, y: %f", waypoint.pose.position.x, waypoint.pose.position.y);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint at x: %f, y: %f", waypoint.pose.position.x, waypoint.pose.position.y);
                return;
            }
        }

        // 모든 경유지를 방문한 후 LED를 끄는 요청을 보냅니다.
        control_led(false);
    }

    // LED 제어 함수
    void control_led(bool turn_on)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = turn_on; // LED 상태 설정 (true: 켜기, false: 끄기)

        // 비동기적으로 서비스 요청을 보내고 결과를 처리
        auto future = led_client_->async_send_request(request);
        handle_service_response<std_srvs::srv::SetBool>(future.share(), turn_on ? "LED control - ON" : "LED control - OFF");
    }

private:
    // 경유지 좌표를 생성하는 함수
    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double theta)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";    // 좌표계는 "map"으로 설정
        pose.header.stamp = this->now(); // 현재 시간을 타임스탬프로 설정

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        // 회전을 위한 쿼터니언 설정
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        return pose;
    }

    // 특정 경유지로 이동하는 함수
    bool navigate_to_pose(const geometry_msgs::msg::PoseStamped &pose)
    {
        // 액션 서버가 준비되기를 기다림
        if (!navigate_action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        // 목표 메시지 생성
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;

        // 액션 목표 전송 옵션 설정
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // 서버가 목표를 수락했는지 확인하는 콜백
        send_goal_options.goal_response_callback = [](const GoalHandleNavigateToPose::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
            }
        };

        // 목표로 이동하는 동안 피드백을 받는 콜백
        send_goal_options.feedback_callback = [](GoalHandleNavigateToPose::SharedPtr,
                                                 const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance to goal: %f meters", feedback->distance_remaining);
        };

        // 결과 콜백 함수 설정
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
            }
        };

        // 목표 전송 및 결과 대기
        auto goal_result_future = navigate_action_client_->async_send_goal(goal_msg, send_goal_options);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
            return false;
        }

        auto goal_handle = goal_result_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        return true; // 목표가 성공적으로 전송되고 실행되었음을 반환
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
            .detach(); // 스레드를 분리하여 비동기 처리
    }

    // 액션 클라이언트와 서비스 클라이언트
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_action_client_; // navigate_to_pose 액션 클라이언트
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr led_client_;            // LED 제어를 위한 서비스 클라이언트
    std::thread service_thread_;                                              // 서비스 요청을 처리하는 스레드
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<WaypointsServiceClient>(options);

    // 프로그램 시작 시 LED를 켜고, 경유지로 이동한 후 LED를 끕니다.
    node->control_led(true);   // LED 켜기
    node->move_to_waypoints(); // 경유지로 이동
    node->control_led(false);  // LED 끄기

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
    WaypointsServiceClient 클래스는 로봇이 설정된 경유지로 이동하도록 명령을 보내고, 이동이 시작될 때 LED를 켜고 완료되면 LED를 끄는 역할을 합니다.
    move_to_waypoints 함수는 여러 경유지를 순차적으로 이동하도록 명령합니다.
    navigate_to_pose 함수는 nav2_msgs::action::NavigateToPose 액션을 사용하여 특정 위치로 로봇을 이동시킵니다.
    control_led 함수는 LED를 켜거나 끄는 서비스 요청을 처리합니다.
    service_thread_는 비동기 작업을 처리하기 위해 별도의 스레드를 사용합니다.

    이 노드는 로봇이 설정된 경유지로 이동하도록 명령을 보내고, 이동이 시작될 때 LED를 켜고 이동이 완료되면 LED를 끄는 기능을 제공합니다.
    이 노드에서는 경유지 좌표를 설정하고, 로봇을 해당 좌표로 이동시키는 작업을 수행합니다.
*/