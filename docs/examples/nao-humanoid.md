# NAO Humanoid Robot

This example demonstrates bipedal humanoid robot control using the SoftBank NAO with MATLAB/Simulink.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Bipedal Humanoid Robot |
| **Robot** | SoftBank NAO V6 |
| **Difficulty** | Advanced |
| **Control Method** | Joint Control / Gait Planning |
| **DOF** | 25 (full body) |

---

## Features

- **Bipedal Walking**: Dynamic gait generation
- **Upper Body Control**: Arm gestures
- **Multi-Modal Sensing**: Cameras, microphones, touch
- **Balance Control**: IMU-based stabilization
- **25 DOF**: Full body articulation

---

## Specifications

| Property | Value |
|----------|-------|
| Height | 574 mm |
| Weight | 5.48 kg |
| DOF | 25 |
| Sensors | Cameras, IMU, FSR, Touch |

---

## Walking Control

- **ZMP-based Gait**: Zero Moment Point control
- **Balance Controller**: Real-time stability
- **Foot Trajectory**: Cycloid-based swing

---

## Quick Start

1. **Open Webots** and load `examples/nao_humanoid/worlds/nao_walking.wbt`
2. **Configure MATLAB** controller
3. **Run simulation** for walking demo
4. **Tune gait parameters** in Simulink

---

## References

- [SoftBank NAO](https://www.softbankrobotics.com/emea/en/nao)
- [Webots NAO](https://cyberbotics.com/doc/guide/nao)
