# Khepera IV Mobile Robot

This example demonstrates the Khepera IV mobile robot with MATLAB/Simulink. The Khepera IV is a compact differential drive robot widely used in robotics research and education.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Differential Drive Mobile Robot |
| **Robot** | K-Team Khepera IV |
| **Difficulty** | Beginner to Intermediate |
| **Control Method** | Differential Drive / PID |
| **Sensors** | IR proximity, Ultrasonic, Camera |

---

## Features

- **Compact Design**: 140mm diameter
- **IR Proximity Sensors**: 8 sensors for obstacle detection
- **Ultrasonic Sensors**: 5 sensors for distance measurement
- **Camera**: Color camera for vision tasks
- **Odometry**: Wheel encoders for position estimation
- **WiFi Communication**: Wireless data transfer

---

## Specifications

| Property | Value |
|----------|-------|
| Diameter | 140 mm |
| Height | 58 mm |
| Weight | 560 g |
| Max Speed | 0.8 m/s |
| Wheel Diameter | 43.5 mm |
| Encoder Resolution | 2764 ticks/revolution |

---

## Sensors

| Sensor | Webots Name | Quantity |
|--------|-------------|----------|
| IR Proximity | `proximity sensor 0-7` | 8 |
| Ultrasonic | `ultrasonic sensor 0-4` | 5 |
| Camera | `camera` | 1 |
| Left Encoder | `left wheel sensor` | 1 |
| Right Encoder | `right wheel sensor` | 1 |

---

## Quick Start

1. Load `examples/khepera_iv/worlds/khepera_arena.wbt`
2. Configure MATLAB controller
3. Run simulation
4. Observe navigation behavior

---

## References

- [K-Team Khepera IV](https://www.k-team.com/khepera-iv)
- [Webots Khepera IV](https://cyberbotics.com/doc/guide/khepera4)
