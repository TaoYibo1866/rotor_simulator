## Build
```Bash
$ sudo apt-get install ros-kinetic-joint_state_publisher_gui
$ cd ~/catkin_ws/src
$ git clone https://gitee.com/harbin-institute-of-technology-csc/rotor_simulator.git
$ cd ~/catkin_ws
$ catkin_make
```
## Run
查看模型
```Bash
$ roslaunch rotor_description display.launch
```
激光武器手柄控制
```Bash
$ roslaunch rotor_gazebo laser_weapon_maunal.launch
```
激光武器视觉伺服
```Bash
$ roslaunch rotor_gazebo laser_weapon_auto.launch
```