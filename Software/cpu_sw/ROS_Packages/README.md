# ROS Packages


## 1. Introduction
This folder contains ready to use package, customized and new package.


## 2. Install ROS packages 

## 2.1 From Package Manager:
### 2.1.1 Lidar Sensor
sudo apt-get install -y ros-ROSDISTRO-rplidar-ros

### 2.1.2 Razor IMU 9DOF Sensor
sudo apt-get install -y ros-ROSDISTRO-razor-imu-9dof

### 2.1.3 Hector Mapping
sudo apt-get install ros-ROSDISTRO-hector-slam

where: ROSDISTRO e.g. kinetic, melodic, etc..


## 2.2 From Source: Clone into project workspace and build it.
During on-going development, I prefer this one as it gives me better flexibility
in case I need to modify something as per my project needs.

### 2.2.1 Lidar Sensor
https://github.com/Slamtec/rplidar_ros

### 2.2.2 Razor IMU 9DOF Sensor
https://github.com/ENSTABretagneRobotics/razor_imu_9dof

### 2.2.3 Hector Mapping
https://github.com/tu-darmstadt-ros-pkg/hector_slam

### 2.2.4 teleop_twist_keyboard - used to control robot's movements manually. 
It is very usefull when testing the motors and when building a map.
https://github.com/ros-teleop/teleop_twist_keyboard

