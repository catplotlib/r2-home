# r2-home

![Robot demo](assets/robot-demo.gif)

A scratch-built mobile robot: custom CAD, 3D printed parts, embedded control, and ROS2 integration.

## What This Project Is

I have been building my own robot from the ground up:

- Designed parts in CAD
- Printed and assembled custom mechanical components
- Wired and integrated Raspberry Pi + Arduino control
- Connected sensors and started the software stack in ROS2

The goal is a capable, autonomous mobile robot that can map and navigate real environments.

## Current Status

Working today:

- Pi and Arduino talking over a serial protocol
- LiDAR integrated and publishing scans
- Calibrated wheel odometry on `/odom`, with the `odom -> base_link` transform
- Closed-loop wheel velocity control (feedforward + PI), so `cmd_vel` is real m/s
- Voice control via Whisper transcription and GPT intent parsing
- Animated face on the robot's display, driven by what it's doing

## What Is Next

- Bring up `slam_toolbox` mapping and record map runs
- Validate odometry against LiDAR over longer drives
- Autonomous navigation with Nav2
- Wheel upgrade for more traction
- Move runtime config paths into `config/`

## Repository Layout

- `ros2/src/motor_driver/` - Serial bridge to the Arduino, odometry + TF, velocity
  control, teleop GUI, odometry monitor. Also holds the robot URDF, the bringup
  launch file, and `firmware/` (the Arduino sketch it talks to).
- `ros2/src/voice_cmd/` - Wake-word listening, speech-to-intent, and the pygame
  face that reacts to what the robot is doing.
- `config/` - Runtime configuration (SLAM parameters).
- `cad/` - CAD models and printable part designs
- `assets/` - Demo images and project visuals

### ROS graph

| Node | Subscribes | Publishes |
|---|---|---|
| `serial_drive_node` | `/cmd_vel`, `/reset_odom` | `/odom`, `odom -> base_link` TF |
| `teleop_gui_node` | - | `/cmd_vel` |
| `odom_monitor_node` | `/odom` | - |
| `voice_cmd_node` | - | `/cmd_vel`, `/robot_awake` |
| `display_node` | `/cmd_vel`, `/robot_awake` | - |
