# iRobot® Create® 3 LIDAR SLAM demo

This example sets up LIDAR SLAM with a Create® 3 robot and Slamtec RPLIDAR spinning laser rangefinder.

## Parts List

* Raspberry Pi® 4[^1]
* USB-C® to USB-C® cable[^2]
* Slamtec RPLidar A1M8
* USB Micro B to USB A cable
* 4x M2.5 x 8 flat head machine screw
* 4x M2.5 x 6 self-tapping screw
* 10x M3 x 10 self-tapping screw

## Setup

### Hardware Setup

The files in this example assume the RPLIDAR is mounted 12 mm behind the center of rotation on the top of the Create 3 robot, in the arrangement shown below.
The SLAM solver relies on a proper `tf` tree; if you wish to mount the sensor in another location, you will need to modify the parameters in the static transform publisher launched from `launch/sensors_launch.py`.

Note that all STLs of brackets referenced in this example are found in our [create3_docs](https://github.com/iRobotEducation/create3_docs/tree/main/docs/hw/data/brackets) repository.

![Image of Create 3 showing setup and placement of sensors](https://iroboteducation.github.io/create3_docs/examples/data/create3_lidar_top.jpg)

1. Affix a LIDAR to your robot.
   STLs are available to mount an RPLidar A1M8 as well as its USB adapter.
   The LIDAR can be attached to the mounting plate using qty. 4 of M2.5 x 8 screws.
   The USB adapter can be attached to its mounting plate by heat staking (we used a soldering iron with an old tip in a well-ventilated space).
   Both mounting plates can be attached to the Create® 3 faceplate with M3 x 10 self-tapping screws.

1. Affix a single board computer (abbreviated as SBC for this guide) to your robot.
   In this example, we use a Raspberry Pi® 4, but other devices should work.
   STLs are available to mount a Raspberry Pi® to the Create® 3 robot.
   The Raspberry Pi® can be attached to the mounting plate using qty. 4 of M2.5 x 6 self-tapping screws.
   The mounting plate can be attached to the Create® 3 cargo bay with M3 x 10 self-tapping screws.

1. Connect the LIDAR to the SBC with a USB Micro B to USB A cable.
   Thread the cable through the passthrough in the top of the robot.

1. Connect the SBC to the Create® 3 adapter board with a USB-C® to USB-C® cable.

### SBC Setup

On the SBC, clone and build the [create3_examples repository](https://github.com/iRobotEducation/create3_examples). 

Source the setup shell scripts in **EVERY** terminal you open:

```bash
source ~/create3_examples_ws/install/local_setup.sh 
```

Run the sensors launch script, which includes the LIDAR driver and transform from the laser scan to the robot:

```bash
ros2 launch create3_lidar_slam sensors_launch.py
```
If your robot is using a namespace, you should add `namespace:='ROBOTNAMESPACE'` to the previous command, where `ROBOTNAMESPACE` is the namespace of your robot

In a separate terminal run slam toolbox:

```bash
ros2 launch create3_lidar_slam slam_toolbox_launch.py
```
If your robot is using a namespace, you should add `namespace:='ROBOTNAMESPACE'` to the previous command, where `ROBOTNAMESPACE` is the namespace of your robot.

There may be some warnings and errors on startup, but the following message will be printed once everything is ready:

```bash
[async_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]
```

In a third terminal, drive the robot around:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
If your robot is using a namespace, you should add `--ros-args -r __ns:=/ROBOTNAMESPACE` to the previous command, where `ROBOTNAMESPACE` is the namespace of your robot.

### Computer Setup

Clone and build the [create3_examples repository](https://github.com/iRobotEducation/create3_examples). 
Then source the setup shell scripts.

```
source ~/create3_examples_ws/install/local_setup.sh
```


Run the rviz launch script, which launches rviz2 with the appropriate configuration:

```bash
ros2 launch create3_lidar_slam rviz_launch.py
```
If your robot is using a namespace, you should add `namespace:='ROBOTNAMESPACE'` to the previous command, where `ROBOTNAMESPACE` is the namespace of your robot.

The rviz2 configuration this command imports will configure rviz2 to subscribe to the laser, the occupancy map, and display the `base_footprint` tf frame that the laser is building off of from the map frame.

![Image of rviz with custom configuration](https://iroboteducation.github.io/create3_docs/examples/data/create3_lidar_rviz.png)




## Tips and Tricks

* Limit rotation speed for best results.


### Troubleshooting

* Ensure the robot, SBC, and computer are all on the same network, using the same middleware.
* If using CycloneDDS, and you are using multiple network interfaces on either the SBC or the computer, be sure to set up your [XML profile(s)](https://iroboteducation.github.io/create3_docs/setup/xml-config/) properly.
* Check your tf tree and make sure it has all the same transforms as [this sample tree](TF_Tree.pdf).

[^1]: Raspberry Pi® is a trademark of Raspberry Pi Trading.
[^2]: USB-C® is a trademark of USB Implementers Forum.
[^3]: All other trademarks mentioned are the property of their respective owners.
