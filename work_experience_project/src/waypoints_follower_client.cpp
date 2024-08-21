#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class WaypointsFollower : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    WaypointsFollower(const rclcpp::NodeOptions &options) : Node("waypoints_follower_client", options)
    {
        _action_client = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        _timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&WaypointsFollower::send_goal, this));
    }

private:
    void send_goal()
    {
        _timer->cancel(); // 타이머를 취소하여 한 번만 실행되도록 설정

        if (!_action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.44, -0.158, 1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.43, 0.353, 3.14));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.122, 0.465, -1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.0, 0.0, 0.0));

        RCLCPP_INFO(get_logger(), "Sending goal request");

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&WaypointsFollower::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&WaypointsFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&WaypointsFollower::result_callback, this, std::placeholders::_1);

        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

    geometry_msgs::msg::PoseStamped get_pose_from_xy_theta(const float &x, const float &y, const float &theta)
    {
        geometry_msgs::msg::PoseStamped pose;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

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

    void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                           const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", feedback->current_waypoint);
    }

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

    rclcpp_action::Client<FollowWaypoints>::SharedPtr _action_client;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsFollower>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
