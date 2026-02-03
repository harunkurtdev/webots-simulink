# Pirana USV

![Pirana USV](./worlds/.ocean.jpg)

The Pirana USV is an unmanned surface vehicle simulation featuring steerable propulsion for marine robotics applications.

## Features

- Steerable propeller propulsion system
- GPS and IMU-based navigation
- Realistic water simulation with fluid dynamics
- Obstacle avoidance scenarios (oil barrels)

## Sensors

| Sensor | Device Name |
|--------|-------------|
| Accelerometer | `accelerometer` |
| Gyroscope | `gyro` |
| GPS | `gps` |
| Inertial Unit | `inertial_unit` |

## Motors

| Motor | Device Name | Function |
|-------|-------------|----------|
| Steering | `rotational_motor` | Propeller direction |
| Propeller | `rotational_motor_propeller` | Thrust |

## Controllers

### pirana_controller
Python-based keyboard controller for manual operation.

### simulink_control_app
Simulink integration for autonomous control algorithms.

## Quick Start

1. Open `worlds/ocean.wbt` in Webots
2. Set controller to `simulink_control_app`
3. Open `simulink_control.slx` in MATLAB
4. Run simulation

## File Structure

```
pirana_usv/
├── controllers/
│   ├── pirana_controller/       # Keyboard control (Python)
│   └── simulink_control_app/    # Simulink integration
└── worlds/
    └── ocean.wbt               # Ocean world with obstacles
```

See [full documentation](../../docs/examples/pirana-usv.md) for detailed information.
