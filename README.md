# ROS RaspberryPi Robot/AGV  

Lidar guided autonomous robot.  


## Hardware  

* Raspberry Pi 3  
* L298N dual H-Bridge  
* Ydlidar X4  
* LiPo 3.7V JST PH 2.0

### Wiring  
H-Bridge | Raspberry Pi GPIO
--- | ---: 
N1 | 13 (PWM1)
N2 | 19 (PWM1)
N3 | 18 (PWM0)
N4 | 12 (PWM0)

<img alt="Raspi 3 PWM pins" src="doc/raspberry_pi_3_PWM_pins.png" width="50%">
<img alt="H-bridge wiring" src="doc/wiring_hbrige_raspi.jpg"  width="50%">

[//]: <> (https://funduino.de/nr-34-motoren-mit-h-bruecke-l298n-ansteuern)

## Setup  

* Ubuntu 18.04 LTS
* ROS Melodic  
* `$ sudo ./ros_install.sh`  

![TF tree](doc/frames.pdf)  

<img alt="rqt_graph" src="doc/rqt_graph.svg" width="100%">


## Start robot  
```
 $ catkin_make  
 $ source /devel/setup.bash  
 $ roslaunch raspi_ctrl raspi.launch  
```
