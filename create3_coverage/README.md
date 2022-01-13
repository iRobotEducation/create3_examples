# iRobot® Create® 3 Coverage

This example creates a ROS 2 action server that runs a simple non-systematic coverage algorithm on your Create® 3.

The purpose of this example is to show how to command your robot and react to its hazard information.

### How to use

Build this and the `create3_examples_msgs` packages.
Source the setup shell scripts.

Start the coverage action server

```bash
ros2 run create3_coverage create3_coverage
```

In a separate terminal command a coverage action

```bash
ros2 action send_goal /coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"
```

### Robot initial configuration

**NOTES:**
 - Do not start the behavior with the robot undocked, but very close to the dock. The behavior may fail or it may cause the robot to run over its dock.
 It's safe to start with the robot still docked.
 - Do not start the behavior with the robot in contact with obstacles or cliffs.


### Troubleshooting

##### `Waiting for an action server to become available...`

If users notice that they are unable to communicate with the coverage action server when using Fast-DDS RMW, this is due to the following bug https://github.com/ros2/rmw_fastrtps/issues/563.
A simple fix consists in updating the `rwm_fastrtps_cpp` library to a version >= 5.0.1
