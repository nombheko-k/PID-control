# PID Tank Sim

A ROS 2 Python node that simulates and visualizes a PID-controlled drone's vertical altitude. The Tkinter GUI shows the drone and setpoint, while a Matplotlib plot records altitude over simulation time. The package is named `pid_tank_sim`; the current simulated plant is a drone-altitude model.

## Requirements

- ROS 2 Jazzy (or a compatible ROS 2 distribution)
- Python 3 with `rclpy`, `std_msgs`, Tkinter, and Matplotlib
- A graphical desktop session for the Tkinter window

Install package dependencies with `rosdep` from the ROS workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/pid_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pid_tank_sim
source install/setup.bash
```

Replace `jazzy` with the name of your installed ROS 2 distribution if needed. Source both setup files in every new terminal before using the package.

## Run

```bash
ros2 run pid_tank_sim sim_node
```

If `Kp`, `Ki`, or `Kd` are not supplied, the node prompts for them in the GUI. To provide gains and run the simulation faster than wall-clock time:

```bash
ros2 run pid_tank_sim sim_node --ros-args \
  -p Kp:=2.0 -p Ki:=0.1 -p Kd:=1.0 \
  -p realtime_factor:=3.0
```

The default simulated duration is 12 seconds. A `realtime_factor` of `3.0` targets completion in about 4 seconds of wall-clock time; actual duration depends on system and rendering load. The final plot remains visible until the window is closed. This package provides a console executable, not a `ros2 launch` launch file.

## Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `dt` | `0.02` | Fixed physics step in simulated seconds |
| `realtime_factor` | `1.0` | Simulated seconds per wall-clock second |
| `mass` | `1.0` | Drone mass in kilograms |
| `gravity` | `9.81` | Gravitational acceleration in m/s^2 |
| `drag_coeff` | `0.3` | Linear vertical drag coefficient |
| `thrust_max` | `20.0` | Maximum thrust in newtons |
| `max_time` | `12.0` | Simulation duration in simulated seconds |
| `max_altitude` | `50.0` | Upper altitude limit in meters |
| `setpoint` | `20.0` | Target altitude in meters |
| `Kp`, `Ki`, `Kd` | prompted | PID gains; can be supplied as ROS parameters |

## ROS interface

- Publishes `std_msgs/msg/Float32` altitude in meters on the relative topic `altitude` (normally `/altitude` in the root namespace).

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
