# Create3 Coverage

This example creates a ROS 2 action server that allows to run a simple, non-systematic, coverage algorithm on your Create3.

The main purpose is to show how to command your robot and how to react to its hazard information.

### How to use

Build this and the `create3_examples_msgs` packages.
Source the setup shell scripts.

Either place your robot on its dock or far from it (note: the behavior may fail or make the robot run over the dock if you start very close to it).

Start the coverage action server

```bash
ros2 run create3_coverage create3_coverage
```

In a separate terminal command a coverage action

```bash
ros2 action send_goal /coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_duration:{sec: 1000,nanosec: 0}}"
```
