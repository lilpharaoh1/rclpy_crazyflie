# rclpy_crazyflie

## High-Level

This library is made so as to developers an easy "plug-and-play" way to interact with the Crazyflie API using ROS2. This repo should be especially useful to those who are new to ROS2. The intention of the library is to operate similarly to other pkgs such as [ros_can](""). Essentially a user can just subscribe to logging topics and publish simple drive commands to the robot, with the library handling all the interfacing with the Crazyflie API. 

The Crazyflie server ( `rclpy_crazyflie/crazyflie_server` ) handles connection, logging and control of the drone swarm. The Crazyflie client ( `rclpy_crazyflie/crazyflie_client` ) handles requests from the user to the Crazyflie server. The Crazyflie RViz package ( `rclpy_cf_rviz` ) simply handles the conversion of internal Crazyflie messages ( `cf_msgs` ) to more RViz-compatible ROS2 messages. It also handles how these messages are displayed. 

## Setup

## Hardware
* Crazyflie 2.X 
* CrazyRadio

## Software
* Python 3.8+
* cflib
* ROS2 Distro ( Galactic+ )
* Oracle VM VirtualBox ( optional )

## Installation
A VM image for this project can be found at [here](""). This image has all the required packages pre-installed and setup. Export this image to Oracle VM VirtualBox preferably.
If you would prefer to set up the packages yourself, the setup instructions for cflib can be found [here]("") and the ROS2 Galactic installation guide can be found [here]("").

## General ROS2 advice ( for beginners )
Everytime you open a new terminal and would like to use ROS2 commands, you will need to source the ROS2 setup.bash file. This can be done by using the command below.
```bash
source /opt/ros2/galactic/setup.bash # TODO: Check this path
```
You can check if this command has worked by typing `ros2 -h` into the command line. If a help guide appears in the terminal, ROS2 has been sourced correctly.
If you would not like to source ROS2 everytime, the source command can be added to you `.bashrc`. This can be done as such.
```bash
vim ~/.bashrc
# Go to the bottom of this file and press i
# Add the line below
source /opt/ros2/galactic/setup.bash
# press esc, followed by :wq and enter
```

Another useful tip is to remember to build when you make a change to your workspace or src files. This can be done by going to the root of your workspace and using the `colcon build` command. After building, you will need to `source` the build for the changes to "take effect". For the VM image, use the following commands.
```bash
cd ~/crazyflie_ws
colcon build --symlink-install
source install/setup.bash
```
The `--symlink-install` tag adds the src files to your build directory. This means you will not need to rebuild every time you change a python src file. However, if you change something that's not a src file, you will need to build again ( such as a launch file, setup.py, etc. )
Additionally, if you have changed only one package and would not like to build your whole workspace again, you can use the `--packages-select` command to build a specific package only.
```bash
colcon build --symlink-install --packages-select [PACKAGE NAME]
source install/setup.bash
```

Finally, if you would like to check that packages or executables you've built have been built correctly, you can use the following commands.
```bash
ros2 pkg list
# or
ros2 pkg executables
```
If your packages built fine but don't appear here it likely means you either did not `source` your build or have not configured your `setup.py` file correctly.

## How to Use

Before launching the library, check the `.json` files found at `~/crazyflie_ws/src/rclpy_crazyflie/data`. In `uris.json`, ensure the URIs of all the drones in your system have been added to this file in the correct format. Use the CFClient to check the URIs of your Crazyflies before use. Similarly, check the `info.json` file to configure what information you want logged and displayed in RViz.

To use the library, ensure you are in the the `crazyflie_ws` and use the following commands.
```bash
cd ~/crazyflie_ws
colcon build --symlink-install 
source install/setup.bash
ros2 launch rclpy_crazyflie crazyflie_swarm.launch.py
```

Wait a few seconds for all of the Crazyflies to connect. Once this is done, you can see the data that is being output by the loggers by using the following commands. 
```bash
ros2 topic list
ros2 topic echo [TOPIC-NAME]
```

You can also launch one of the examples in another terminal
```bash
cd ~/crazyflie_ws
source install/setup.bash
ros2 launch rclpy_cf_examples [EXAMPLE-LAUNCH-FILE] # ex. position_control
```

Finally, you can visualise the movement of data through active nodes by using `rqt_graph`. This can be done by using the following command and navigating the GUI.
```bash
ros2 run rqt_graph rqt_graph
```
