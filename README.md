# ROS RaspberryPi Robot/AGV  

Lidar guided autonomous robot.  


## Hardware  

* Raspberry Pi 3  
* L298N dual H-Bridge  
* Ydlidar X4  
* LiPo 3.7V JST PH 2.0

## Setup  

* Ubuntu 18.04 LTS
* ROS Melodic  
* `$ sudo ./ros_install.sh`  

[TF tree](doc/frames.pdf)  

[rqt_graph](dic/rqt_graph.svg)  

## Start  

```
 $ catkin_make  
 $ source /devel/setup.bash  
 $ roslaunch raspi_ctrl raspi.launch  
```
