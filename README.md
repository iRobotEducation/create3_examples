# create3_examples

Example nodes to drive the iRobot® Create® 3 Educational Robot.

### Dependencies

Make sure that ROS 2 Humble is already installed in your system.
You can follow the [official instructions](https://docs.ros.org/en/humble/Installation.html).

### Build instructions

First, source your ROS 2 workspaces with all the required dependencies.
Then, you are ready to clone and build this repository.
You should only have to do this once per install.

```sh
mkdir -p create3_examples_ws/src
cd create3_examples_ws/src
git clone https://github.com/iRobotEducation/create3_examples.git
cd ..
rosdep install --from-path src --ignore-src -yi
colcon build
```

### Initialization instructions

You will have to do this in every new session in which you wish to use these examples:

```sh
source ~/create3_examples_ws/install/local_setup.sh
```

### Run the examples

Refer to the individual examples README.md for instructions on how to run them.

### Potential pitfalls

If you are unable to automatically install dependencies with rosdep (perhaps due to [this issue](https://github.com/ros-infrastructure/rosdep/issues/733)), please do be sure to manually install the dependencies for your particular example of interest, contained in its package.xml file.