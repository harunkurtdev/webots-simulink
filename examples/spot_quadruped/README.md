# Spot Quadruped Robot

This example demonstrates quadruped robot control inspired by Boston Dynamics Spot with MATLAB/Simulink. The simulation includes legged locomotion, terrain navigation, and dynamic balance control.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Quadruped Robot |
| **Difficulty** | Advanced |
| **Control Method** | Gait Control / MPC |
| **DOF** | 12 (3 per leg) |
| **Sensors** | IMU, Depth cameras, Joint encoders |

---

## Features

- **Dynamic Walking**: Trot, walk, bound gaits
- **Terrain Adaptation**: Uneven ground navigation
- **Balance Control**: Real-time stability
- **12-DOF Leg Control**: Hip, knee, ankle joints
- **Depth Sensing**: Environment perception
- **Autonomous Navigation**: Path planning

---

## Project Structure

```
spot_quadruped/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m
│       ├── simulink_control.slx
│       ├── gait_scheduler.m
│       ├── leg_kinematics.m
│       ├── balance_controller.m
│       └── wb_*.m
└── worlds/
    └── spot_terrain.wbt
```

---

## Leg Configuration

| Leg | Hip (Abduction) | Hip (Flexion) | Knee |
|-----|-----------------|---------------|------|
| Front Left | `front left shoulder abduction` | `front left shoulder rotation` | `front left elbow` |
| Front Right | `front right shoulder abduction` | `front right shoulder rotation` | `front right elbow` |
| Rear Left | `rear left shoulder abduction` | `rear left shoulder rotation` | `rear left elbow` |
| Rear Right | `rear right shoulder abduction` | `rear right shoulder rotation` | `rear right elbow` |

---

## Gait Patterns

### Trot Gait
- Diagonal leg pairs move together
- Good balance and speed
- Most common walking gait

### Walk Gait
- One leg moves at a time
- Maximum stability
- Slow movement

### Bound Gait
- Front legs together, rear legs together
- High speed capability
- Dynamic motion

---

## Quick Start

1. Load `examples/spot_quadruped/worlds/spot_terrain.wbt`
2. Configure MATLAB controller
3. Run simulation
4. Observe quadruped locomotion

---

## References

- Boston Dynamics Spot
- Raibert, M. (1986). "Legged Robots That Balance"
- Bledt, G. et al. (2018). "MIT Cheetah 3: Design and Control"
