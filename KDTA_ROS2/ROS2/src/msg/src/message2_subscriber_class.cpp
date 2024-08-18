#include "msg/m2sub.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Message2Subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}