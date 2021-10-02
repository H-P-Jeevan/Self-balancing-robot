# Self-balancing-robot

In this project we deal with the design and control of a self balancing robot. The robot consists of 2 wheels and a base_link. It consists of imu sensors that are used to detect orientation.  

---

### Table of Contents

- [Installation](#installation)
- [Technologies](#technologies)
- [How To Use](#how-to-use)
- [References](#references)
- [Author Info](#author-info)

---

## Installation

- Use these commands to install ROS Melodic on Ubuntu 18.04: 
    > sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    > sudo apt install curl  
    > curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  
    > sudo apt update  
    > sudo apt install ros-melodic-desktop-full

- Install git
    > apt-get install git

- Creating catkin workspace: 
    > source /opt/ros/melodic/setup.bash  
    > mkdir -p ~/catkin_ws/src  
    > cd ~/catkin_ws/  
    > catkin_make  
    > source devel/setup.bash  
 
 - Cloning the github repository
    > cd ~/catkin_ws/src/  
    > git clone https://github.com/H-P-Jeevan/Self-balancing-robot.git
 
---

## Technologies

- ROS

---

## How To Use

#### Running gazebo simulation
  > roslaunch model5_description gazebo.launch 

#### Running differnt controllers
 - Run non linear control system 
    > rosrun model_gazebo controller.py 

---

## References
Gazebo plugins
   > http://gazebosim.org/tutorials?tut=ros_gzplugins

Gazebo joints
   > http://wiki.ros.org/urdf/XML/joint

---

## Author Info

- H P Jeevan https://www.linkedin.com/in/h-p-jeevan-08607a1a8
