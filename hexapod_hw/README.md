# hexapod_hw package

This package contains configuration files for controlling the hexapod drives using [`dynamixel_motor`][dynamixel_motor]. This stack provides controller management functionality in a similar spirit to the [`ros_control`][ros_control] framework.

To start controlling a chain of Dynamixel servos start by running the controller manager with

```bash
roslaunch hexapod_hw controller_manager.launch
```

To start individual joint position controllers for each of the servos on the leg, use

```bash
roslaunch hexapod_hw start_leg_1_controller.launch
```

Note that configuration parameters are spread amongst the above two launch files and the [`controllers.yaml`](config/controllers.yaml) configuration file.

[dynamixel_motor]: http://wiki.ros.org/dynamixel_motor
[ros_control]: http://wiki.ros.org/ros_control
