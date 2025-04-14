## Build
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

#### Find ACM Port
> ls /dev | grep ttyACM

## To launch
> ros2 launch qube_bringup bringup.launch.py simulation:=<true/false> device:=</dev/ttyACM0> baud_rate:=<115200>
>
#### Controller
> ros2 launch qube_controller launch.py 

##### Change desired RPM
Preferably use this to launch service client:
> ros2 run qube_controller_msgs qube_controller_client_node

Optionally:
> ros2 param set /qube_controller_node reference <reference>

##### Change regulator gains
> ros2 param set /qube_controller_node kp <kp>

> ros2 param set /qube_controller_node ki <ki>

> ros2 param set /qube_controller_node kd <kd>


