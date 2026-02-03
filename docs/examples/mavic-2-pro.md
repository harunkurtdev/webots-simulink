# Mavic 2 Pro Drone

This example demonstrates quadrotor drone control using the DJI Mavic 2 Pro with MATLAB/Simulink.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Quadrotor UAV |
| **Robot** | DJI Mavic 2 Pro |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | 6-DOF Flight Control / PID |

---

## Features

- **6-DOF Flight Control**: Full attitude and position control
- **GPS Navigation**: Waypoint following and position hold
- **Obstacle Avoidance**: Front/bottom range sensors
- **Camera Gimbal**: 2-axis stabilized camera
- **Autonomous Flight**: Takeoff, landing, waypoint missions

---

## Specifications

| Property | Value |
|----------|-------|
| Diagonal Size | 354 mm |
| Weight | 907 g |
| Max Speed | 20 m/s |
| Motors | 4 (X configuration) |
| Sensors | IMU, GPS, Camera, Range sensors |

---

## Control Architecture

- **Position Controller**: GPS-based position hold
- **Attitude Controller**: Roll, pitch, yaw control
- **Rate Controller**: Angular rate control
- **Motor Mixing**: X-configuration mixer

---

## Quick Start

1. **Open Webots** and load `examples/mavic_2_pro/worlds/mavic_flight.wbt`
2. **Configure MATLAB** path
3. **Run simulation** for autonomous flight demo
4. **Customize waypoints** in Simulink model

---

## References

- [DJI Mavic 2 Pro](https://www.dji.com/mavic-2)
- [Webots Mavic](https://cyberbotics.com/doc/guide/mavic-2-pro)
