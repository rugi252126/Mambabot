# Mambabot

## 1. Introduction
Mambabot is an autonomous driving car. The named "Mambabot" is derived from the nickname of the late basketball super star Kobe Bryant.
In a way, this project is also a tribute to him.

The project is in state of "proof of concept". For now, I will mainly use the prototype for my personal study and research.

The prototype will be built out from my existing hardware and electronic components.
The base control unit is powered by STM32 M4 cortex microcontroller. It mainly handles the motor control, hall sensor, IMU, GPS, transceiver,
calibration and so on. The car has also Lidar sensor and two cameras.
The central processing unit or the high power computer is powered by Nvidia Jetson Tx2 running with Ubuntu OS. All heavy data processing
will be done here. And as for the framework, I will be using ROS (Robot Operating System).

I will provide the complete BOM and schematic diagram once the concept is finalized.
In the future, other information (e.g. designs, source codes, etc..) will be available here as well.

As I have knowledge and experience on the other part of the system, to make this project a success, my study will focus more on Deep learning for computer vision for image classification.
My study includes but not limited to the following.
1. Libraries and Packages
2. Parameterized Learning (Data, Scoring Function, Loss Function, Weights and Biases)
3.1. Optimization Methods
   - Gradient Descent(Standard vanilla implementation, SGD(Stochastic Gradient Descent)
3.2. Regularization
   - Different type of regularization techniques
4. Tuning-in the hyperparameters(e.g. learning rate, Weights(W))
5. NN (Neutral Network)/ ANN (Artificial Neutral Network)
   - Dataset
   - Loss function
   - Model/Architecure
   - Optimization Method
5. CNN (Convolutional Neural Networks)
6. Training and testing
7. Saving and Loading the Models
8. Spotting Underfitting and Overfitting
   
   
Deep learning is indeed a huge topic and for that reason, the following consideration will be taken into account.
1. The car will be deployed first inside the house. Having said that, the image classification will focus on those things present inside the house like table and chair.
2. Supervised learning will be used. For now, unsupervised and semi-supervised learning are out of scope.
3. As the project and my research go on, the coverage will be increased.

