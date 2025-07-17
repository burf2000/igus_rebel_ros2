# igus ReBeL ROS node #

## Summary ##

* Connects to an igus ReBeL over an Ethernet connection

## Compatibility ##

This was tested on ROS2 Jazzy on Ubuntu 24.04.2.

## Installation ##

Currently, the package can only be used by building it from source.

* Create a colcon workspace (or use an existing one)
* Clone (or download) this repository into the `src` folder of the colcon workspace
* Navigate to the colcon workspace folder and install the dependencies with `rosdep install --from-paths . --ignore-src -r -y`
* Build with `colcon build`

To control the robot with MoveIt, or simulate with Gazebo, you should also install the following ROS packages:

* joint_state_broadcaster 
* gz_ros2_control

In Ubuntu, install with `sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster`

## Usage ##

The ros node expects to reach the robot at the IP and port `192.168.3.11:3920`

It is recommended to run the ROS node with the provided launch file, using `ros2 launch igus_rebel rebel.launch.py`

To control the robot with MoveIt, first start the igus_rebel ROS node (`ros2 launch igus_rebel rebel.launch.py`) and then run `ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true`

To simulate the robot in Gazebo and control the simulated robot with MoveIt run `ros2 launch igus_rebel_moveit_config igus_rebel_simulated.launch.py`

## docker ##

A docker container with this ROS node is available in the 'docker' branch of this repository.

## Set digital outputs

The ReBeL's digital outputs can be set with a call to the service `/set_digital_output`. 

The service input is a `DigitalOutput` message, which is defined as
```
int8 output
bool is_on
```

- `output` is the index of the output whose state should be set.
- `is_on` is the state to which the output should be set. `True` means on, `False` means off.

The service output is defined as
```
bool success
string message
```
`success` is always True, `message` is always empty.
