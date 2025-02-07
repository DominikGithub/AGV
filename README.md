# AGV  

Lidar guided autonomous, mobile robot based on ROS.  

[//]: # "[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE)"

## Start  
```
 $ catkin_make  
 $ source /devel/setup.bash  
 $ roslaunch raspi_ctrl raspi.launch  
```

### Remote RVIZ client  
```
export ROS_MASTER_URI=http://<ROBOT IP>:11311/
export ROS_IP=<REMOTE RVIZ CLIENT IP> 
```
see `/rviz/setup_ros_env.sh`  
```
$ roslaunch raspi_ctrl rviz.launch
```  

Manual keyboard control via [Teleop twist keyboard](http://wiki.ros.org/teleop_twist_keyboard)  

### Lidar SLAM  
<img alt="ydlidar_screen" src="doc/ydlidar_rviz.png" width="75%">

### ROS TF  
![TF tree](doc/frames.pdf)  
<img alt="rqt_graph" src="doc/rqt_graph.png" width="100%">

## Setup  
* [Ubuntu](https://wiki.ubuntu.com/ARM/RaspberryPi) 18.04 for ARM  
* ROS Melodic  
* `$ sudo ./ros_install.sh`  

## Hardware  
* Raspberry Pi 3  
* L298N dual H-Bridge  
* Ydlidar X4  
* Adafruit PowerBoost 1000  
* LiPo 3.7V  
* 2x 9V battery  

### Motor wiring  
H-Bridge | Raspi GPIO | Motors  
--- | :--- | :---
IN1 | 13 (PWM1) |  
IN2 | 19 (PWM1) |  
IN3 | 18 (PWM0) |  
IN4 | 12 (PWM0) |  
OUT1 | | M1 forward  
OUT2 | | M1 backward  
OUT3 | | M2 forward  
OUT4 | | M2 backward  

<img alt="Raspi 3 PWM pins" src="doc/raspberry_pi_3_PWM_pins.png" width="50%">
<img alt="H-bridge wiring" src="doc/wiring_hbrige_raspi.jpg"  width="50%">

[//]: <> (https://funduino.de/nr-34-motoren-mit-h-bruecke-l298n-ansteuern)

## Robot assembly  
<img alt="robot complete" src="doc/photos_assembly/20201101_130622.jpg" width="50%">
<img alt="robot complete" src="doc/photos_assembly/20201101_130505.jpg" width="50%">
<img alt="robot complete" src="doc/photos_assembly/20201101_130325_croped.jpg" width="50%">

## FreeCAD designs  

Top power & control | Lidar   
--|-- 
<img alt="cad hbridge" src="doc/cad_designs/RaspiRobot_raspi_base_hbridge.png" width="75%"> | <img alt="cad lidar" src="doc/cad_designs/RaspiRobot_ydliard_mounting_plate_v2.png" width="75%">  
Bottom power  |
<img alt="cad battery" src="doc/cad_designs/RaspiRobot_battery_bottom.png" width="75%"> | 

# Project structure  
```
├── README.md
├── cad
│   └── README.md
├── doc
│   ├── cad_designs/
│   ├── photos_assembly/
│   ├── frames.pdf
│   ├── raspberry_pi_3_PWM_pins.png
│   ├── rqt_graph.svg
│   ├── ydlidar_rviz.png
│   └── wiring_hbrige_raspi.jpg
├── ros_install.sh
└── src
    ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
    ├── motor_pkg
    │   ├── CMakeLists.txt
    │   ├── bin
    │   │   └── motor_ctrl
    │   ├── package.xml
    │   ├── setup.py
    │   └── src
    │       └── motor_ctrl
    │           ├── __init__.py
    │           ├── motor.py
    │           └── name_ascii.txt
    ├── raspi_ctrl
    │   ├── CMakeLists.txt
    │   ├── launch
    │   │   ├── costmap_configs
    │   │   │   ├── base_local_planner_params.yaml
    │   │   │   ├── costmap_common_params.yaml
    │   │   │   ├── global_costmap_params.yaml
    │   │   │   └── local_costmap_params.yaml
    │   │   ├── mapping_default.launch
    │   │   ├── raspi.launch
    │   │   ├── rviz.launch
    │   │   └── x4_lidar.launch
    │   ├── package.xml
    │   └── rviz
    │       ├── rviz.rviz
    │       └── setup_ros_env.sh
    ├── teleop_twist_keyboard
    └── ydlidar_ros
```
