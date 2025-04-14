# Mechatronics and robotics mini project
![image](https://github.com/user-attachments/assets/3586e122-58a5-49c4-9915-2eee1f8963b8 =250x250)


## How to setup project
> mkdir -p project_name/src
>
> cd project_name/src
>
> git clone https://github.com/LiamFlsvik/Mekatronikk_mini_prosjekt.git
>
> cd Mekatronikk_mini_prosjekt/
>
> git submodule update --init --recursive
>
> cd ../../
>
> colcon build
>
> source install/setup.bash

Furthermore you can connect to the qube or set the simulation argument to true:

#### Find ACM Port
> ls /dev | grep ttyACM

## Launching the visualization
> ros2 launch qube_bringup bringup.launch.py simulation:=<true/false> device:=</dev/ttyACM0> baud_rate:=<115200>
>
#### Controller
> ros2 launch qube_controller launch.py 

##### Change desired RPM
Preferably use this to launch service client:
> ros2 run qube_controller_msgs qube_controller_client_node

Optionally:
> ros2 param set /qube_controller_node reference value

##### Change regulator gains
Proporsional gain:
> ros2 param set /qube_controller_node kp value
Integral gain:
> ros2 param set /qube_controller_node ki value
Derivative gain:
> ros2 param set /qube_controller_node kd value


