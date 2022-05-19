[![License badge](https://img.shields.io/badge/license-Apache2-green.svg)](http://www.apache.org/licenses/LICENSE-2.0)
[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

# Computer Vision using ROS2

This project contains code examples created in Visual Studio Code for Computer Vision using C++ & OpenCV & Point Cloud Library (PCL) using ROS2. These examples are created for the Computer Vision Subject of Robotics Software Engineering Degree at URJC.

This package allows running different Gazebo worlds, including the [AWS Robomaker](https://github.com/aws-robotics) worlds, using the Tiago robot from [PAL Robotics](https://github.com/pal-robotics)


# Installation

Execute installation script:
```bash
./setup.sh
``` 

# Run Gazebo & Tiago in ROS2

```bash
source install/setup.sh
ros2 launch computer_vision sim.launch.py
``` 

To change the Gazebo world or the initial position/rotation of the Tiago robot, you can modify the `config/params.yaml` file.

# Run examples in ROS2

* OpenCV node
```bash
source install/setup.sh
ros2 run computer_vision cv_node
``` 

* PCL node

```bash
source install/setup.sh
ros2 run computer_vision pcl_node
``` 

* OpenCV & PCL node

```bash
source install/setup.sh
ros2 run computer_vision pcl_node
``` 

# Help

You can check the `commands` file to run other nodes, such as `key_teleop` to move Tiago using your keyboard.

## About

This is a project made by [José Miguel Guerrero], Assistant Professor at [Universidad Rey Juan Carlos].
Copyright &copy; 2022.

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
