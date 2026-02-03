# E-puck Swarm Robotics

This example demonstrates swarm robotics using multiple E-puck robots with MATLAB/Simulink control.

![E-puck Swarm](../assets/images/epuck/epuck_robot.png)

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Swarm Robotics / Multi-Robot System |
| **Robot** | E-puck (8 instances) |
| **Difficulty** | Intermediate |
| **Control Method** | Decentralized / Behavior-based |

---

## Features

- **Multiple Robot Coordination**: 8 E-puck robots working together
- **Decentralized Control**: Each robot runs independently
- **Swarm Behaviors**: Aggregation, dispersion, flocking
- **Proximity Sensing**: 8 IR sensors per robot
- **Inter-Robot Communication**: IR-based local communication

---

## E-puck Specifications

| Property | Value |
|----------|-------|
| Diameter | 70 mm |
| Height | 50 mm |
| Max Speed | 0.13 m/s |
| Sensors | 8 IR proximity, Camera, Accelerometer |
| Communication | IR emitter/receiver |

---

## Swarm Behaviors

### Aggregation
Robots gather in a common area based on local sensing.

### Dispersion
Robots spread out evenly in the environment.

### Flocking
Reynolds rules: separation, alignment, cohesion.

---

## Quick Start

1. **Open Webots** and load `examples/epuck_swarm/worlds/swarm_arena.wbt`
2. **Configure MATLAB** as the controller
3. **Run simulation** to observe collective behavior
4. **Modify behavior** parameters in Simulink model

---

## References

- [E-puck Official](http://www.e-puck.org/)
- [Webots E-puck](https://cyberbotics.com/doc/guide/epuck)
- Reynolds, C. W. (1987). "Flocks, herds and schools"
