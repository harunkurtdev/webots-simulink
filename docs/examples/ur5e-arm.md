# UR5e Robot Arm

This example demonstrates industrial robot arm control using the Universal Robots UR5e with MATLAB/Simulink.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | 6-DOF Collaborative Robot Arm |
| **Robot** | Universal Robots UR5e |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | Joint Space / Cartesian / Trajectory |

---

## Features

- **6 Degrees of Freedom**: Full spatial manipulation
- **Joint Position Control**: Individual joint angle control
- **Inverse Kinematics**: Cartesian position to joint angles
- **Trajectory Planning**: Smooth motion profiles
- **Collaborative**: Safe human-robot interaction

---

## Specifications

| Property | Value |
|----------|-------|
| Weight | 20.6 kg |
| Payload | 5 kg |
| Reach | 850 mm |
| DOF | 6 |
| Repeatability | ±0.03 mm |

---

## Kinematics

- **Forward Kinematics**: Joint angles to end-effector pose
- **Inverse Kinematics**: End-effector pose to joint angles
- **Jacobian**: Velocity mapping

---

## Quick Start

1. **Open Webots** and load `examples/ur5e_arm/worlds/ur5e_workspace.wbt`
2. **Configure MATLAB** controller
3. **Run simulation** for pick-and-place demo
4. **Program trajectories** in Simulink

---

## References

- [Universal Robots UR5e](https://www.universal-robots.com/products/ur5-robot/)
- [Webots UR5e](https://cyberbotics.com/doc/guide/ure)
