# Computer Vision using ROS2

This project contains code examples created in Visual Studio Code for Computer Vision using C++ & OpenCV & Point Cloud Library (PCL) using ROS2. These examples are created for the Computer Vision Subject of Robotics Software Engineering Degree at URJC.

This package allows running different Gazebo worlds, including the [AWS Robomaker](https://github.com/aws-robotics) worlds, using the Tiago robot from [PAL Robotics](https://github.com/pal-robotics)


# Installation

Source your ROS2 distro:
```bash
source /opt/ros/<ros2-distro>/setup.bash
```

Execute installation script:
```bash
./setup.sh
``` 

# Run Gazebo & Tiago in ROS2

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 launch computer_vision simulation.launch.py
``` 

To change the Gazebo world or the initial position/rotation of the Tiago robot, you can modify the `config/params.yaml` file.

If you have a low performance, close the Gazebo's client. Check gzclient process, and kill it:
```bash
kill -9 `pgrep -f gzclient`
``` 

# Run Navigation in ROS2

You can use [Nav2] using Tiago in the selected world:

```bash
source install/setup.sh
ros2 launch computer_vision tiago_navigation.launch.py
``` 
Also, you can use [Keepout Zones], just create a new map including the excluded areas, and use the same name adding `_keep`, now publish the map running:

```bash
source install/setup.sh
ros2 launch computer_vision keepzone.launch.py
``` 

Just some AWS worlds are included. You can [Navigate While Mapping] and create your own map using the [SLAM Toolbox] provided. In different terminals:

* Run the SLAM Toolbox:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=install/slam_toolbox/share/slam_toolbox/config/mapper_params_online_async.yaml use_sim_time:=true
```
In slam_toolbox/config/mapper_params_online_async.yaml change scan_topic from /scan to /scan_raw

* Activate the map server:

```bash
ros2 launch nav2_map_server map_saver_server.launch.py
```

* Check the map in RViz:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

* Save the map:

```bash
ros2 run nav2_map_server map_saver_cli --ros-args -p use_sim_time:=true
```

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
ros2 run computer_vision cv_pcl_node
``` 

# Help

You can check the `commands` file to run other nodes, such as `key_teleop` to move Tiago using your keyboard.

## About

This is a project made by [José Miguel Guerrero], Associate Professor at [Universidad Rey Juan Carlos].
Copyright &copy; 2022.

[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

## License

Shield: 

[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg


[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
[Nav2]: https://navigation.ros.org/
[Keepout Zones]: https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html?highlight=keep
[SLAM Toolbox]: https://vimeo.com/378682207
[Navigate While Mapping]: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
