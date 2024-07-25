#Usage Guide

This package requires Ros2 Humble Hawksbill 


Set up your environment:
```bash
cd Indy7/mpc
source /opt/ros/humble/setup.bash
```

You can change parameters in mpc_settings.hpp

Build:
```bash
colcon build
```

Source the install directory
```bash
source install/setup.bash
```

Run robot_driver:
```bash
ros2 run mpc robot_driver
```

Then run trajectory_publisher:
```bash
ros2 run mpc trajectory_publisher
```


