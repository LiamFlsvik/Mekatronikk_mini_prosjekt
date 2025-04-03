#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
class qube_simulator{
public:

qube_simulator(): last_time(std::chrono::steady_clock::now()){}


void update(){    
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time-last_time).count();
    last_time = current_time;

    //angle = theta(s)/V(s) = K/(s(Ts+1)), solved in statespace:

    x_1 += x_2*dt;
    x_2 += (-1/T*x_2+K/T*voltage) * dt;
}

//getters
double get_angle(){
    return x_1;
}
double get_velocity(){
    return x_2;
}
double get_noise(){
    return noise;
}
double get_K(){
    return K;
}
double get_T(){
    return T;
}
double get_voltage(){
    return voltage;
}

//setters
void set_noise(const double noise_){
    noise = noise_;
}
void set_K(const double K_){
    K = K_;
}
void set_voltage(const double voltage_){
    voltage = voltage_;
}
void set_T(const double T_){
    T = T_;
}





private:
double K = 230; //rad
double T = 0.15; //rad/s
double voltage{0}, noise{0};
double x_1{10}, x_2{1};
std::chrono::steady_clock::time_point last_time;



};