# Spot Quadruped Robot

This example demonstrates quadruped robot control inspired by Boston Dynamics Spot with MATLAB/Simulink.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Quadruped Robot |
| **Difficulty** | Advanced |
| **Control Method** | Gait Control / MPC |
| **DOF** | 12 (3 per leg) |

---

## Features

- **Dynamic Walking**: Trot, walk, bound gaits
- **Terrain Adaptation**: Uneven ground navigation
- **Balance Control**: Real-time stability
- **12-DOF Control**: Hip, knee, ankle per leg

---

## Gait Patterns

- **Trot**: Diagonal leg pairs
- **Walk**: One leg at a time
- **Bound**: Front/rear pairs together

---

## Quick Start

1. **Open Webots** and load `examples/spot_quadruped/worlds/spot_terrain.wbt`
2. **Configure MATLAB** controller
3. **Run simulation** for locomotion demo

---

## References

- Boston Dynamics Spot
- Raibert, M. (1986). "Legged Robots That Balance"
