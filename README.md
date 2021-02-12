# 转台模拟器
## 编译
```Bash
$ sudo apt-get install ros-kinetic-joint-state-publisher-gui
$ cd ~/catkin_ws/src
$ git clone https://gitee.com/harbin-institute-of-technology-csc/rotor_simulator.git
$ git clone https://gitee.com/harbin-institute-of-technology-csc/drone_simulator.git
$ cd ~/catkin_ws
$ catkin_make
```
## 运行结果
查看模型
```Bash
$ roslaunch rotor_description display.launch
```
![image](rotor2_rviz.png)
激光武器手柄控制
```Bash
$ roslaunch rotor_simulator laser_weapon_maunal.launch
```
![image](laser_weapon_manual.png)
激光武器视觉伺服
```Bash
$ roslaunch rotor_simulator laser_weapon_auto.launch
```
