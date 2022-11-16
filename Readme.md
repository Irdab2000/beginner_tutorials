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

### cppcheck command
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint command
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```

