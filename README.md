![GitHub Release](https://img.shields.io/github/v/release/Task-Intelligent-Robotics-Research-Grp/artros)
![GitHub License](https://img.shields.io/github/license/Task-Intelligent-Robotics-Research-Grp/artros)

| ROS 2 Distribution | Jazzy                                                                                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/artros/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/artros/actions/workflows/jazzy-build.yaml) |

artros
==================================================

## Overview


## Installation
First, you should install a developer package of C++ library,  `nlohmann-json3`, for handling `JSON` messages in `C++` code;
```bash
sudo apt install nlohmann-json3-dev
```
Then download [this package](https://github.com/Task-Intelligent-Robotics-Research-Grp/artros) as well as its dependencies into the ROS2 workspace;
```bash
cd ros2_ws/src
git clone https://github.com/Task-Intelligent-Robotics-Research-Grp/artros
vcs import . --input=artros/dependencies.repos
```
Finally, you can compile the package by typing
```bash
source ros2_ws/install/setup.bash
colcon build
```

## Quick start


## More info.
Please refer to the following pages.
- [Documentation of artros](https://task-intelligent-robotics-research-grp.github.io/artros/index.html): User manual and API reference manual.
- [Documentation of artros_msgs](https://task-intelligent-robotics-research-grp.github.io/artros/aist_msgs/index.html): Definitions of ROS message/service/action used in `artros`.
