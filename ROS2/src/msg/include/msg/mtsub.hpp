#ifndef MTSUB_HPP
#define MTSUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class MessageTimeSubscriber : public rclcpp::Node
{
public:
    MessageTimeSubscriber();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub1;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub2;
    void sub_message_msg(const std_msgs::msg::String::SharedPtr msg);
    void sub_time_msg(const std_msgs::msg::Header::SharedPtr msg);
};

#endif