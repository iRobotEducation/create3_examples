# Create3 Teleoperation

This package contains scripts and instructions for teleoperating the the Create® 3 robot using keyboard or joystick.

### Keyboard Teleoperation

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Joystick Teleoperation

```sh
ros2 launch create3_teleop teleop_joystick_launch.py
```

This will default to an xbox 360 controller, but can be easily overriden using the `joy_config` launchfile argument for any of the supported platforms. As of time of writing, these are:
- Logitech Attack3 (`atk3`)
- Logitech Extreme 3D Pro (`xd3`)
- PS3 (`ps3` or `ps3-holonomic`)
- Xbox 360 (`xbox`)

Example for a PS3 controller:

```sh
ros2 launch create3_teleop teleop_joystick_launch.py joy_config:=ps3
```

Also, it's possible to select the specific device to use with the `joy_dev` argument. It can be used as follows:

```sh
ros2 launch create3_teleop teleop_joystick_launch.py joy_dev:=/dev/input/js1
```
