#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pid_controller.hpp"
#include "qube_controller_msgs/srv/set_reference.hpp"



#include <chrono>
#include <functional>
#include <vector>

class qube_controller_node : public rclcpp::Node {
public:
    qube_controller_node() : Node("qube_controller_node"),
        pid_controller_(1.0, 0.01, 0.00),
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
            std::chrono::milliseconds(20),
            std::bind(&qube_controller_node::computeAndPublish, this));
        service_ = this->create_service<qube_controller_msgs::srv::SetReference>(
            "set_reference",
            std::bind(&qube_controller_node::set_reference_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        declare_parameter("kp", pid_controller_.get_kp());
        declare_parameter("ki", pid_controller_.get_ki());
        declare_parameter("kd", pid_controller_.get_kd());
        declare_parameter("reference", reference_);

        parameter_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&qube_controller_node::parameterCallback, this, std::placeholders::_1));
    }
private:
    void set_reference_callback(
        const std::shared_ptr<qube_controller_msgs::srv::SetReference::Request> request,
        std::shared_ptr<qube_controller_msgs::srv::SetReference::Response> response) {
        
        if (request->request >= -400 && request->request <= 400) {
            this->reference_ = request->request;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Reference is set to: %.2f", reference_);
            pid_controller_.set_reference(request->request);
        } else {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Invalid value: %.2f (must be between:-400 and 400)", request->request);
        }
        
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(!msg->name.empty() && msg->name[0] == "motor_joint"){

            if(!msg->position.empty()){
                measured_angle_ = msg->position[0];
            }
            if(!msg->velocity.empty()){
                measured_velocity_ = msg->velocity[0]; 
            }
            filtered_velocity = alpha*filtered_velocity + (1-alpha)*measured_velocity_;
        }
    }

    void computeAndPublish()
    {
        double control_signal = pid_controller_.update(reference_, filtered_velocity);

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.push_back(control_signal);  
        publisher_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Reference: %.2f | Measured velocity: %.2f | Output: %.2f",
                    reference_, filtered_velocity, control_signal);
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
    double measured_angle_{0}, measured_velocity_{0}, filtered_velocity{0}, alpha{0.5};
    rclcpp::Service<qube_controller_msgs::srv::SetReference>::SharedPtr service_;
};
