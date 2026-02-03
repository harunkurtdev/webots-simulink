# Firebird 6 Mobile Robot

This example demonstrates the Firebird 6 robot with MATLAB/Simulink. The Firebird 6 is a versatile mobile robot platform with extensive sensor arrays for research and education.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Mobile Research Platform |
| **Robot** | Nex Robotics Firebird 6 |
| **Difficulty** | Intermediate |
| **Control Method** | Differential Drive / PID |
| **Sensors** | IR, Sharp distance, Encoders, IMU |

---

## Features

- **8 IR Proximity Sensors**: Obstacle detection ring
- **3 Sharp Distance Sensors**: Front and sides
- **Wheel Encoders**: Precise odometry
- **IMU**: Orientation sensing
- **Line Sensors**: Line following array
- **Gripper Option**: Object manipulation
- **Wireless**: XBee communication

---

## Specifications

| Property | Value |
|----------|-------|
| Diameter | 195 mm |
| Height | 115 mm |
| Weight | ~1.5 kg |
| Max Speed | 0.5 m/s |
| Encoder Resolution | 300 CPR |

---

## Sensors

| Sensor | Webots Name | Quantity |
|--------|-------------|----------|
| IR Proximity | `ps0-7` | 8 |
| Sharp Distance | `sharp front/left/right` | 3 |
| Left Encoder | `left wheel sensor` | 1 |
| Right Encoder | `right wheel sensor` | 1 |
| Accelerometer | `accelerometer` | 1 |
| Gyroscope | `gyro` | 1 |

---

## Quick Start

1. Load `examples/firebird6/worlds/firebird_arena.wbt`
2. Configure MATLAB controller
3. Run simulation
4. Implement navigation algorithms

---

## Applications

- Mobile robot research
- SLAM implementation
- Multi-robot systems
- Educational robotics

---

## References

- [Nex Robotics Firebird](https://www.nex-robotics.com/products/fire-bird-robots/)
