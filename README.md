# edu_fleet_sim
This package comprises a pygame-based robot simulation for multiple robots organized on one or multiple fleets.

![Screenshot of Robot Simulator](/images/screenshot.png)

Tests have been performed with ROS melodic, albeit the used python versions differ. While ROS melodic uses python2.x, the simulator need python3. The  reason is, that ROS noetic will require python3. In order to make the simulator work with ROS melodic, you can follow the installation hints below.

## Prerequisites

This package needs following dependencies to be installed.
* A ROS1 distribution. Please pick a release from the [ROS Website](http://wiki.ros.org/ROS/Installation). However this package was tested with [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
*   pygame, that could be installed via apt if you on a Debian/Ubuntu system, or using pip that should work in any case:

    ```console
    ~$ sudo apt install python3-pygame
    ```
    or
    ```console
    ~$ pip3 install --user pygame
    ```

* To control the robots or fleets the ROS package [edu_virtual_joy](https://github.com/EduArt-Robotik/edu_virtual_joy) is recommended. In this case the provided launch file (see below) could be used. However the alternative is to use [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message to control the robots. Or the [joy messages](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html), where the three first axes are used:
    * axes[0] == x,
    * axes[1] == y,
    * axes[2] == omega

Is the alternative option used a control message for each robot fleet has to be provided. The topic names are:
    * fleet1/cmd_vel
    * fleet2/cmd_vel
    * overall_fleet/cmd_vel

## Building and Installing Package

To build the package, simply catkin has to been called in your root directory of the workspace where the edu_fleet_sim was installed. Note: if you are new to ROS please see this [Website](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) for further information.

```console
~/<workspace>/$ catkin_make install
~/<workspace>/$ source install/setup.bash # maybe necessary if not done by some script like .bashrc
```

## Starting the Simulator

Note: if you are new to ROS, please make sure that the install/setup.bash has been sourced in your workspace (see above). Also important: before a ROS node can be started, the ROS master must be running. This happens automatically when roslaunch is used, but not when rosrun is used.

The application could been started by an ROS launch file including a control application, or as node where some parameter has to be set manually. Both options are described below:

### Using rosrun Command

The application is started as ROS node with the following command:

```console
~/rosrun edu_fleet_sim fleet_sim_node
```

The topics of the ROS node are adjusted by following command:

```console
~/rosrun edu_fleet_sim fleet_sim_node fleet1/cmd_vel:=<your custom topic> fleet2/cmd_vel:=<your custom topic> overall_fleet:=<your custom topic>
```

### Using Prepared Launch File

```console
~/roslaunch edu_fleet_sim fleet_sim_demo.launch
```
