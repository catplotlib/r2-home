# r2-home

![Robot demo](assets/robot-demo.gif)

An ongoing scratch-built robot project: custom CAD, 3D printed parts, embedded control, and ROS2 integration.

> **Work in Progress**
>
> This repo documents the build as it happens. Some systems are stable, some are experimental, and a few are still held together by hope and zip ties.

## What This Project Is

I have been building my own robot from the ground up:

- Designed parts in CAD
- Printed and assembled custom mechanical components
- Wired and integrated Raspberry Pi + Arduino control
- Connected sensors and started the software stack in ROS2

The goal is a capable, autonomous mobile robot that can map and navigate real environments.

## Current Status

Hardware and basic integration are behaving:

- ✅ Pi is talking to Arduino
- ✅ ROS2 sees the LiDAR
- ✅ Wheel encoders are publishing usable data
- ✅ Odometry is calibrated and published as `/odom` + `odom -> base_link` TF
- ✅ Closed-loop wheel velocity control (feedforward + PI), so `cmd_vel` is real m/s
- ✅ Voice control via Whisper + GPT intent parsing, with an animated face on the display
- 🚧 SLAM bring-up with `slam_toolbox` (launched, still being tuned)

## What Is Next

### Immediate

- Tune the SLAM pipeline and record clean mapping runs
- Validate odometry + LiDAR consistency over longer drives
- Move the hard-coded config paths (`/home/puja/*.yaml`) into the repo's `config/`

### In Progress

- Wheel upgrade (current setup works, but needs better traction and reliability)
- Voice recognition improvements (usable, but still rough)

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

## Notes

This is not a finished platform yet, but it is moving in the right direction.
