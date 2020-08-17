# Mambabot

## 1. Introduction
Mambabot is an autonomous driving car. The named "Mambabot" is derived from the nickname of the late basketball super star Kobe Bryant.

The project is in state of "proof of concept". For now, I will mainly use the prototype for my personal study and research.

The prototype is build out from my existing hardware and electronic components.
The base control unit is powered by STM32 M4 cortex microcontroller. It mainly handles the motor control, hall sensor, IMU, GPS, transceiver,
calibration and so on.
And the central processing unit is powered by Nvidia Jetson Tx2 running with Ubuntu OS. The communication with different components is done via ROS framework.
The car has Lidar sensor and two cameras. The first camera is placed in front of the car and the other one at the back.

I will be using Deep learning for computer vision for image classification using the front camera. 
At the early stage of the development, the back camera will mainly use OpenCV for object detection.

Deep learning image classification is a huge topic and for that, the following consideration will be taken into account.
1. The car will be deployed first inside the house. Having said that, the image classification will focus on those things present inside the house like table and chair.
2. Supervised learning will be used. For now, unsupervised and semi-supervised learning are out of scope.
3. As the project and my research go on, the coverage will be increased.