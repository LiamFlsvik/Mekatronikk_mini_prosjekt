#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>

class pid_controller{
    public:
    pid_controller(double kp_, double ki_, double kd_): kp(kp_), ki(ki_), kd(kd_), i(0), last_time(std::chrono::steady_clock::now()) {}  

    // PID controller
    double update(double reference, double measured_angle){
        // Calculate the difference in time since the last update
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time-last_time).count();
            last_time = current_time;
        
            error = reference-measured_angle;
            double p = kp*error;
            i += ki*error*dt;
        // Integral limit    
            if(i>i_max){i = i_max;}
            if( i<-i_max){i = -i_max;}
        // Derivative term with low-pass filter
            double d = kd*(error-last_error)/dt;
            filtered_d = alpha*filtered_d + (1-alpha)*d;
            last_error = error;
            
            double pid_output = p+i+filtered_d;
        // Limit the output to the range [-max_pid, max_pid]
            if(pid_output>max_pid){pid_output = max_pid;}
            if(pid_output<-max_pid){pid_output = -max_pid;}
            return pid_output;
    }
    //setters

    void set_reference(const double reference_){
        reference = reference_;
    }

    void set_kp(const double kp_){
        kp = kp_;
    }
    void set_ki(const double ki_){
        ki = ki_;
    }
    void set_kd(const double kd_){
        kd = kd_;
    }

    void set_pid_constants(const double kp_, const double ki_, const double kd_){
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
    //getters
    double get_kp(){
        return kp;
    }
    double get_ki(){
        return ki;
    }
    double get_kd(){
        return kd;
    }
    double get_reference(){
        return reference;
    }
    
private:
double reference{0}, setpoint{0};
double kp{0}, ki{0}, kd{0}, i{0}, filtered_d{0}, alpha{0.9}, last_error{0}, error{0}, i_max{200}, max_pid{1000};
std::function<double(double, double)> pid_update;
std::chrono::steady_clock::time_point last_time;
};