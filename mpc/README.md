# Usage Guide

This package requires Ros2 Humble Hawksbill 

## Building

```bash
git clone https://github.com/A2R-Lab/Indy7.git
cd Indy7/mpc
git submodule update --init --recursive
```

Set up your environment:
```bash
source /opt/ros/humble/setup.bash
```

You can change parameters in mpc_settings.hpp and MPCGPU/include/common/settings.cuh

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

Then run trajopt_node:
```bash
ros2 run mpc trajopt_node
```

or trajectory_publisher:
```bash
ros2 run mpc trajectory_publisher
```

