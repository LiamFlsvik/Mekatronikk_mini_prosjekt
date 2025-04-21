#include "rclcpp/rclcpp.hpp"
#include "qube_controller_msgs/srv/set_reference.hpp"
#include <iostream>
#include <memory>

class qube_controller_client : public rclcpp::Node {
public:
qube_controller_client(): Node("qube_controller_client") {
    client_ = this->create_client<qube_controller_msgs::srv::SetReference>("set_reference");
    
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service ");
    }
   // When the service is available, we can start sending requests
    while(rclcpp::ok()) {
        double reference;
        std::cout << "Enter desired RPM: ";
        std::cin >> reference;

        if(reference >= -400 && reference <= 400){
            auto request = std::make_shared<qube_controller_msgs::srv::SetReference::Request>();
            request->request = reference;

            auto result = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Reference is set to: %.2f", reference);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid value: %.2f (only accepts values in the intervall[-400,400])", reference);
        } 
        
    }
}

private:
    rclcpp::Client<qube_controller_msgs::srv::SetReference>::SharedPtr client_;

};