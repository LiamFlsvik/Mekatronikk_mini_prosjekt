#include "qube_simulator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class qube_simulator_node : public rclcpp::Node {
public:
    qube_simulator_node() : Node("joint_simulator_node"), qube_simulator_() {
        command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10,
            std::bind(&qube_simulator_node::setVoltage, this, std::placeholders::_1));

        
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  
            std::bind(&qube_simulator_node::updateState, this));

        
        this->declare_parameter("K", qube_simulator_.get_K());
        this->declare_parameter("T", qube_simulator_.get_T());
        this->declare_parameter("noise", qube_simulator_.get_noise());

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&qube_simulator_node::parameterCallback, this, std::placeholders::_1));
    }

private:
    void setVoltage(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            qube_simulator_.set_voltage(msg->data[0]);
        }
    }

    void updateState() {
        qube_simulator_.update();

        sensor_msgs::msg::JointState state_msg;
        state_msg.header.stamp = this->now();
        state_msg.name.push_back("qube_motor_joint");
        state_msg.position.push_back(qube_simulator_.get_angle());
        state_msg.velocity.push_back(qube_simulator_.get_velocity());

        joint_state_pub_->publish(state_msg);

        RCLCPP_INFO(this->get_logger(), "Voltage: %.2f | Angle: %.2f | Velocity: %.2f",
                    qube_simulator_.get_voltage(),
                    qube_simulator_.get_angle(),
                    qube_simulator_.get_velocity());
    }

    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            if (p.get_name() == "K") qube_simulator_.set_K(p.as_double());
            else if (p.get_name() == "T") qube_simulator_.set_T(p.as_double());
            else if (p.get_name() == "noise") qube_simulator_.set_noise(p.as_double());
        }

        return result;
    }

    qube_simulator qube_simulator_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
