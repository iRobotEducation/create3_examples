# create3_republisher


## Instructions

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
