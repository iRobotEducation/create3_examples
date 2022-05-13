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

1. Affix a LIDAR to your robot.
   STLs are available to mount an RPLidar A1M8 as well as its USB adapter.
   The LIDAR can be attached to the mounting plate using qty. 4 of M2.5 x 8 screws.
   The USB adapter can be attached to its mounting plate by heat staking (we used a soldering iron with an old tip in a well-ventilated space).
   Both mounting plates can be attached to the Create® 3 faceplate with M3 x 10 self-tapping screws.

1. Affix an SBC to your robot.
   In this example, we use a Raspberry Pi® 4, but other devices should work.
   STLs are available to mount a Raspberry Pi® to the Create® 3 robot.
   The Raspberry Pi® can be attached to the mounting plate using qty. 4 of M2.5 x 6 self-tapping screws.
   The mounting plate can be attached to the Create® 3 cargo bay with M3 x 10 self-tapping screws.

1. Connect the LIDAR to the SBC with a USB Micro B to USB A cable.
   Thread the cable through the passthrough in the top of the robot.

1. Connect the SBC to the Create® 3 adapter board with a USB-C® to USB-C® cable.

### SBC Setup

On the SBC, build this and source the setup shell scripts.

Start the sensors launch script, which includes the LIDAR driver and transform from the laser scan to the robot:

```bash
ros2 launch create3_lidar sensors_launch.py
```

In a separate terminal run slam toolbox:

```bash
ros2 launch create3_lidar slam_toolbox_launch.py
```

In a third terminal, drive the robot around:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Computer Setup

On your computer, start rviz2.
(TODO: more to come here)

## Tips and Tricks

Limit rotation speed for best results.

### Troubleshooting

[^1]: Raspberry Pi® is a trademark of Raspberry Pi Trading.
[^2]: USB-C® is a trademark of USB Implementers Forum.
[^3]: All other trademarks mentioned are the property of their respective owners.
