#ifndef MSUB2_HPP
#define MSUB2_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class Message2Subscriber : public rclcpp::Node
{
public:
    Message2Subscriber();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
    void sub_message2_msg(const std_msgs::msg::String::SharedPtr msg);
};

#endif