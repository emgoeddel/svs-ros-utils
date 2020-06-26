# SVS Util

A ROS package with launch scripts, world files, and object files to support
the ROS version of SVS.

## Setup Instructions

Prerequisites:

*  Ubuntu 18.04
*  ROS Melodic
*  Soar (see https://soar.eecs.umich.edu/articles/articles/building-soar)
*  A catkin workspace (see http://wiki.ros.org/catkin/Tutorials)
*  The Fetch ROS packages (fetch_gazebo and fetch_ros)

Instructions:

1. Clone this repo into your catkin workspace.
2. catkin build

You should now be able to start the connector with roslaunch.
Note that there is no code to compile in this package, but you still need to
run catkin build to register the package with ROS for roslaunch to work.

## Usage

First start a Gazebo environment with a simulated Fetch:

```
roslaunch svs_util start_gazebo.launch gui:="<true/false>" world:="<name.sdf>"
```

* gui: enables/disables the Gazebo GUI
* world: provide the name of a file in the svs_util/worlds folder

Then start the Soar debugger as usual. It will start a ROS node and connect to
the relevant topics.