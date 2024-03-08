# create3_republisher

## What is this?

This package contains a ROS 2 applications capable of republishing ROS 2 entities.
It's designed to work with the iRobot Create 3 robot.

This means that if the robot is publishing on topic `/my_robot/odom` this application can subscribe to this topic and republish its messages into a new namespace, e.g. `/my_republisher/odom`.
This works also with publishers, services and actions.

For example, with `/my_robot/cmd_vel` remapped to `/my_republisher/cmd_vel`, users can then send motion commands to `/my_republisher/cmd_vel` to move the robot around, or send a ROS 2 action goal to `/my_republisher/drive_distance`, etc..

## Why using this?

ROS 2 middlewares have a tendency to discover all the ROS 2 entities in the network and allocate resources for all of them, even if they don't need to communicate.
This can cause performance issues due to excessive memory allocations, discovery traffic and double delivery of messages.
This republisher can be used to get around this problem.

**IMPORTANT:** use this tool only if you experience problems in your setup.

**NOTE:** Using this tool requires at least some understanding of how to configure ROS 2 application via DDS XML configuration files.

## Prerequisites

 - Your robot and other ROS 2 applications should be on ROS 2 Humble.

 - You should use Fast-DDS as your RMW on the robot and throughout all your applications.
 This approach is not RMW-specific, but the instructions and the provided DDS config file are.

 - Ensure that you can discover and communicate via ROS 2 between all your devices before starting.
 This procedure will use advanced communication configuration, and if stuff wasn't working before, it will be hard to debug it later.
 These instructions assume that you didn't need any custom DDS configuration to get communication working.
 If that's not the case, your custom configuration will likely need to be integrated with the provided XML config files, which is not covered by this tutorial.

## Connecting the Create 3 Robot and an SBC via the republisher

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

1. Modify the Fast-DDS XML profile on the Create 3, through the webserver, to exactly match the [`fastdds-robot-passive-unicast.xml` profile](dds-config/fastdds-robot-passive-unicast.xml).

1. Launch the republisher app in a terminal on your RaspberryPi (or other SBC) using the provided [`fastdds-active-unicast.xml` profile](dds-config/fastdds-active-unicast.xml).
For example:

    ```bash
    source ~/ws/install/setup.sh
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-active-unicast.xml
    ros2 launch create3_republisher create3_republisher_launch.py
    ```

    This command assumes that the Create 3 robot is using the default namespace and it will republish all Create 3 robot names into the `/repub` namespace.

    If your Create 3 robot is running with a namespace, for example `/my_robot`, and/or you want to change the republication namespace, for example `/my_repub`, you can do it via command line arguments.

    ```bash
    ros2 launch create3_republisher create3_republisher_launch.py republisher_ns:=/my_repub robot_ns:=/my_robot
    ```

    **IMPORTANT: THE TWO NAMESPACES MUST BE DIFFERENT!!**

    **NOTE:** the list of entities that's republished is defined by the `params.yaml` file which you'll find at `~/ws/install/create3_republisher/share/create3_republisher/bringup/params.yaml`.
    Comment or uncomment entries in this file to customize it to your needs.

1. Run your other applicatios that want to communicate with Create 3 in another terminal of the RaspberryPi (or other SBC) using a passive unicast XML profile.
For example:

    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/ws/src/create3_examples/create3_republisher/dds-config/fastdds-passive-unicast.xml
    ros2 topic echo /repub/tf
    ros2 action send_goal /repub/drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"
    ```

1. **OPTIONAL: Connect your laptop and your RaspberryPi (or other SBC)**
With the setup described so far, you should have your RaspberryPi and your Create 3 robot able to communicate via ROS 2.
However, your laptop (or other devices in the network) won't be able to discover neither the robot nor the SBC.
To enable this communication, which again will occurr through the republisher, you will need a DDS configuration file also on your laptop (or other device).
  - Get the Wi-Fi IP address of the RaspberryPi (that's the one you use to SSH into it).
  - Create a copy of the [`fastdds-active-unicast.xml` profile](dds-config/fastdds-active-unicast.xml), which will be referenced as `fastdds-laptop-unicast.xml` in the following steps.
  - Modify the new `fastdds-laptop-unicast.xml` file, replacing the IP address in the line `<address>192.168.186.2</address>` with the RaspberryPi IP address mentioned before.
  For example `<address>192.168.1.212</address>`.

## Connecting the Create 3 Robot and your laptop via the republisher

If you don't have a RaspberryPi or other SBC, you can still use the republisher application on your laptop.
Most of the instructions presented above remain unchaged, but you will not be able to use the provided `fastdds-active-unicast.xml` profile for the republisher process.
This DDS profile contains hardcoded the `usb0` IP address of the Create 3 robot.
You will need to modify it to use the Wi-Fi IP address of the Create 3 instead.
  - Get the Wi-Fi IP address of the Create 3 robot (this is the IP address you use to access the Create 3 webserver).
  - Create a copy of the [`fastdds-active-unicast.xml` profile](dds-config/fastdds-active-unicast.xml), which will be referenced as `fastdds-laptop-unicast.xml` in the following steps.
  - Modify the new `fastdds-laptop-unicast.xml` file, replacing the IP address in the line `<address>192.168.186.2</address>` with the Create 3 IP address mentioned before.
  For example `<address>192.168.1.103</address>`.

Note: these changes to the `fastdds-active-unicast.xml` profile are very similar to what done in the previous instructions to allow the laptop to communicate with the RaspberryPi.
The only, but fundamental, difference is that here we are using the Create 3 IP address, rather than the RaspberryPi IP address.


### Notes and Tips

**IMPORTANT: this republisher is powered by the concept of "unicast discovery" protocols. There must be only 1 DDS profile in your setup that references the Create 3 IP address and it must be used only by the republisher process.
Keep this in mind if you need to do further modifications to the configs, besides what described in this page.**

The "old" Create 3 entity names will not be usable anymore in this configuration.
After running those steps, for example, you won't be able to subscribe to the robot tf topic via `ros2 topic echo /tf` but only via `ros2 topic echo /repub/tf`.

Inspecting the ROS graph (e.g. via `ros2 topic list`) may seem to still show the robot's topics (e.g. you may see `/tf` being listed there), but you can notice that subscribing to those topics won't produce any data.
Indeed, querying more information about the topics will show you that what you are seeing is a "subscriber" (that lives on the republisher) and not the robot's publisher.

```
pi@raspberrypi:~$ ros2 topic info -v /tf
Type: tf2_msgs/msg/TFMessage

Publisher count: 0

Subscription count: 1

Node name: create3_repub
Node namespace: /repub
Topic type: tf2_msgs/msg/TFMessage
Endpoint type: SUBSCRIPTION
GID: 01.0f.e8.52.41.09.d1.75.01.00.00.00.00.00.19.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
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
