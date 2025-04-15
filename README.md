# Mechatronics and robotics mini project



<img src="https://github.com/user-attachments/assets/f857117d-12be-47c5-af38-0ed0798d1f75" width="450">


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

Or:
> ros2 param set /qube_controller_node reference value

##### Change regulator gains
Proportional gain:

> ros2 param set /qube_controller_node kp value
> 
Integral gain:
> 
> ros2 param set /qube_controller_node ki value
> 
Derivative gain:
> 
> ros2 param set /qube_controller_node kd value
> 

## Launch Rviz + GUI
To view and interact with the qube use the following command:
> ros2 launch qube_description view_qube.launch.py

# Videos (coming soon)



