# create3_republisher

## What is this?

This package contains a ROS 2 applications capable of republishing ROS 2 entities.
It's designed to work with the iRobot Create 3 robot.

This means that if the robot is publishing on topic `/my_robot/odom` this node can subscribe to the topic and republish its messages into a new namespace, e.g. `/my_republisher/odom`.
This works also with publishers, services and actions.

With `/my_robot/cmd_vel` remapped to `/my_republisher/cmd_vel`, users can then send motion commands to `/my_republisher/cmd_vel` and move the robot around.
Or send a ROS 2 action goal to `/my_republisher/drive_distance`, etc..

## Why using this?

ROS 2 middlewares have a tendency to discover all the ROS 2 entities in the network and allocate resources for all of them, even if they don't need to communicate.
This republisher can be used to get around this problem.

## Instructions

The following instructions will show how to isolate the robot to discover and communicate only with an entity, the republisher, and ignore all other processes running on your RaspberryPi (or other SBC).
Other entities, e.g. your navigation application can then interact with the republisher (i.e. they can subscribe to Create 3 topics, and send requests) without "being discovered by it".

1. Build this repository on the RaspberryPi (or other SBC).
For example:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/iRobotEducation/create3_examples.git
cd ~/ws
colcon build --symlink-install
```

1. Ensure that the Create 3 robot is running with Fast-DDS.

1. Modify the Fast-DDS XML profile on the Create 3, through the webserver, to match the [`fastdds-robot-passive-unicast.xml` profile](dds-config/fastdds-robot-passive-unicast.xml).

1. Launch the republisher app in a terminal on your RaspberryPi (or other SBC) using the provided [`fastdds-active-unicast.xml` profile](dds-config/fastdds-active-unicast.xml).
For example:

```bash
source ~/ws/install/setup.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-active-unicast.xml
ros2 launch create3_republisher create3_republisher_launch.py
```

This example will republish all Create 3 robot names into the `/repub` namespace.
This example assumes that the Create 3 robot is using the default namespace.

If your Create 3 robot is running with a namespace, for example `/my_robot`, and/or you want to change the republication namespace, for example `/my_repub`, you can do it via command line arguments.

```bash
ros2 launch create3_republisher create3_republisher_launch.py republisher_ns:=/my_repub robot_ns:=/my_robot
```

**IMPORTANT: THE TWO NAMESPACES MUST BE DIFFERENT!!**

1. Run your other applicatios that want to communicate with Create 3 in another terminal of the RaspberryPi (or other SBC) using a unicast XML profile.
For example:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-passive-unicast.xml
ros2 topic echo /repub/tf
ros2 action send_goal /repub/drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"
```

**TIP:**
If you have a ROS 2 application that has hardcoded Create 3 topics and action names (e.g. `/cmd_vel` or `/my_create3/dock`), you don't necessarily need to modify it to use the republisher.
Just change the Create 3 namespace to "something else" and set the republisher namespace as the old robot's namespace.
For example, if you robot has a namespace `/my_create3` you can: first of all change its namespace to `/my_robot` and then run the republisher as:

```bash 
ros2 launch create3_republisher create3_republisher_launch.py republisher_ns:=/my_create3 robot_ns:=/my_robot
```

This approach can be used also if the robot didn't have a namespace to start with!
Assign it to a new namespace and then use `/` as `republisher_ns`.
