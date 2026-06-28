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

Hardware and basic integration are finally behaving:

- ✅ Pi is talking to Arduino
- ✅ ROS2 sees the LiDAR
- ✅ Wheel encoders are publishing usable data

We are **very** close to attempting SLAM.

## What Is Next

### Immediate

- Bring up first SLAM pipeline
- Validate odometry + LiDAR consistency
- Record and review mapping runs

### In Progress

- Wheel upgrade (current setup works, but needs better traction and reliability)
- Voice recognition improvements (usable, but still rough)

## Repository Layout

- `arduino/` - Microcontroller code and hardware interface logic
- `ros2/` - ROS2 packages, launch files, and robot software stack
- `cad/` - CAD models and printable part designs
- `assets/` - Demo images and project visuals

## Notes

This is not a finished platform yet, but it is moving in the right direction.
