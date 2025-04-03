#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pid_controller.hpp"


#include <chrono>
#include <functional>
#include <vector>

class qube_controller_node : public rclcpp::Node {
public:
    qube_controller_node() : Node("qube_controller_node"),
        pid_controller_(0.2, 0.01, 0.01),
        reference_(0.0),
        measured_angle_(0.0),
        measured_velocity_(0.0)    
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);   

        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&qube_controller_node::jointStateCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&qube_controller_node::computeAndPublish, this));

        declare_parameter("kp", pid_controller_.get_kp());
        declare_parameter("ki", pid_controller_.get_ki());
        declare_parameter("kd", pid_controller_.get_kd());
        declare_parameter("reference", reference_);

        parameter_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&qube_controller_node::parameterCallback, this, std::placeholders::_1));
    }
private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(!msg->position.empty()){
            measured_angle_ = msg->position[0];
        }
        if(!msg->velocity.empty()){
            measured_velocity_ = msg->velocity[0]; 
        }
    }

    void computeAndPublish()
    {
        double control_signal = pid_controller_.update(reference_, measured_velocity_);

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.push_back(control_signal);  
        publisher_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Reference: %.2f | Measured velocity: %.2f | Output: %.2f",
                    reference_, measured_velocity_, control_signal);
    }

   

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &p : params) {
            if (p.get_name() == "kp") pid_controller_.set_kp(p.as_double());
            if (p.get_name() == "ki") pid_controller_.set_ki(p.as_double());
            if (p.get_name() == "kd") pid_controller_.set_kd(p.as_double());
            if (p.get_name() == "reference") reference_ = p.as_double();
        }
        return result;
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;

    pid_controller pid_controller_;
    double reference_;
    double measured_angle_, measured_velocity_;
};
