# Thymio II Educational Robot

This example demonstrates the Thymio II robot with MATLAB/Simulink. Thymio is an educational robot designed for learning robotics and programming concepts.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Educational Mobile Robot |
| **Robot** | Mobsya Thymio II |
| **Difficulty** | Beginner |
| **Control Method** | Differential Drive / Reactive |
| **Sensors** | IR proximity, Ground sensors, Accelerometer |

---

## Features

- **Educational Design**: Easy to program and understand
- **7 IR Proximity Sensors**: 5 front, 2 rear
- **2 Ground Sensors**: Line following capability
- **Accelerometer**: Motion detection
- **LED Feedback**: Multi-color visual indicators
- **Sound**: Speaker for audio feedback
- **Buttons**: Touch interface

---

## Specifications

| Property | Value |
|----------|-------|
| Dimensions | 112 x 117 x 53 mm |
| Weight | 270 g |
| Max Speed | 14 cm/s |
| Battery | 3.7V 1500mAh Li-Po |
| Autonomy | ~2 hours |

---

## Sensors

| Sensor | Webots Name | Quantity |
|--------|-------------|----------|
| Front IR | `prox.horizontal.0-4` | 5 |
| Rear IR | `prox.horizontal.5-6` | 2 |
| Ground | `prox.ground.0-1` | 2 |
| Accelerometer | `acc` | 1 |

---

## Pre-programmed Behaviors

Thymio has built-in behaviors (colors):
- **Green**: Friendly - follows hand
- **Yellow**: Explorer - avoids obstacles
- **Red**: Fearful - escapes from obstacles
- **Purple**: Obedient - follows commands
- **Blue**: Attentive - reacts to claps
- **Cyan**: Investigator - follows lines

---

## Quick Start

1. Load `examples/thymio2/worlds/thymio_playground.wbt`
2. Configure MATLAB controller
3. Run simulation
4. Program reactive behaviors

---

## References

- [Thymio Official](https://www.thymio.org/)
- [Webots Thymio II](https://cyberbotics.com/doc/guide/thymio2)
