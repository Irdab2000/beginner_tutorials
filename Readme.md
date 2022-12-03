[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Publisher and Subscriber  ROS2

### Overview

A ROS2 Publisher and Subscriber node with a custom string message.

### Dependencies and Assumptions
- OS : Ubuntu 20.04 
- ROS2 Distro : ROS2 Humble
- Package dependencies : ```rclcpp```, ```std_msgs``` 

### Clone the repository
```
cd <to_your_ws>/src
git clone https://github.com/Irdba2000/beginner_tutorials.git
```

### Build Instructions
```
cd <to_your_ws>/src 
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
source . install/setup.bash
source ~/<your ROS2 installation>/install/local_setup.bash
```

### Run Publisher
To run the publisher node, open a new terminal and run:
```
cd <to_your_ws>
. install/setup.bash
ros2 run beginner_tutorials talker
```
### Run Subscriber
To run the subscriber node, open a new terminal and run:
```
cd <to_your_ws>
. install/setup.bash
ros2 run beginner_tutorials listener
```
### Run client node
To run client node
```
cd<to_your_ws>
. install/setup.bash
ros2 run beginner_tutorials client
```

### Run launch file
Run the launch file first, then run the client node.
To run the launch file:
```
cd <ROS2_ws>/
. install/setup.bash
 ros2 launch beginner_tutorials launcher.yaml 
```
### Command to view the tf frames
To run view_frames, in a new terminal :
```
ros2 run tf2_tools view_frames
```
### Command to print the transformations
In our case,In a new terminal, run :
```
ros2 run tf2_ros tf2_echo world talk
```
### Recording a bag file with all topics
In a new terminal, run :
```
ros2 bag record -a -o ros_bag_tf
```
### Inspecting the bag file
In a new terminal,run :
```
 ros2 bag info ros_bag_tf
 ```

### Playing bag files
In a new terminal,run :
```
ros2 bag play ros_bag_tf
```

### Using launch file to use rosbag
In a new terminal, run :
```
ros2 launch beginner_tutorials launcher.yaml hz:=10.0 bool_record_bag:=true
```
### Playing back the bag file and Listener node
In a new terminal, run:
```
ros2 bag play my_bag --loop
```
In another terminal, run the listener node :
```
ros2 run beginner_tutorials listener
```
### RUnning the test cases
Compile and build the workspace using :
```
colcon build --packages-select beginner_tutorials
```
Then run:
```
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

### cppcheck command
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint command
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```

