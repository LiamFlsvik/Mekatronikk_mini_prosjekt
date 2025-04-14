#include "rclcpp/rclcpp.hpp"
#include "qube_controller_client.hpp"


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<qube_controller_client>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}