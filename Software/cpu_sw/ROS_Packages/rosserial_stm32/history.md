history.md contains the information of changes I made on the ready to use cloned package.

Clone on March 13, 2021
Branch: Master
Link: https://github.com/yoneken/rosserial_stm32


1. How to generate the ROS libraries:
a. Add an "Inc" folder because "ros_lib" will be generated inside this "Inc" folder.
Example, for instance we wanted to generate it in path /home/rudy/mambabot_ws/src
add an "Inc" folder inside "src" then

b. Open a terminal and run below command (path is where the "Inc" folder located).
rudy@rudy-dell:~/mambabot_ws/src$ rosrun rosserial_stm32 make_libraries.py .

c. If we want to generate a custom ROS message as part of the "ros_lib",
first, we have to create the custom message package and compile it.
After that, run the command mentioned in item #2.
The custom ROS message will be generated together with other ROS libraries inside "ros_lib" folder.

2. How to add the custom ROS message in the project:
a. Copy the generated custom ROS message to the project under "ros_lib". Do note that the project only contains the relevant ROS libraries.
b. Add it in the project environment and make it compilable.



