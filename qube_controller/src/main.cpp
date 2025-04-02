#include "qube_controller_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<qube_controller_node>());
    rclcpp::shutdown();
    return 0;
}