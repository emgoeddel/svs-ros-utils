# SVS-ROS Connector

A ROS node that's also an SML Client. Allows the ROS ecosystem
to communicate with Soar through SVS.

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

## Usage

First start a Gazebo environment with a simulated Fetch:

```
roslaunch svs_ros_connector start_gazebo.launch gui:="<true/false>" world:="<name.sdf>"
```

* gui: enables/disables the Gazebo GUI
* world: provide the name of a file in the svs_ros_connector/worlds folder

Then start the connection to Soar:

```
roslaunch svs_ros_connector svs_connector.launch remote_soar:="<true/false>" agent_file:="<name.soar>"
```

* remote_soar: set to true to connect to a Soar agent already running in the debugger;
               set to false to create a new Soar agent
* agent_file: if remote_soar is false, provide the full path to an agent file to load
              when the agent is created

