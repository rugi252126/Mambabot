# ROS Testing

The package "ros_test" is used to test the communication between robot's computer and robot's base(STM32).

Make sure to create a ROS workspace and copy the package "ros_test" then do "catkin_make" to compile the package.
Belo link shows on how to create ROS workspace.
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

There are 3 launch files available inside "launch" folder.
- all_nodes.launch will launch the publisher and subscriber nodes
- talker.launch will launch the publisher node
- listener.launch will launch the subscriber node

Alternatively, without using the launch file, it can be done in this way.
Note: Make sure that each command is executed in a separate terminal
1. roscore
2. rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
3. rosrun ros_test talker
4. rosrun ros_test listener
5. Run "rostopic list" to see the list of topics
