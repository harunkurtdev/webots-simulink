# Scout V2.0 Mobile Robot

![Scout V2.0](./worlds/.world.jpg)

The Scout V2.0 is a four-wheeled mobile robot platform developed by AgileX Robotics for autonomous navigation research and development.

## Features

- Four-wheel differential drive system
- LiDAR sensor integration (LDS-01)
- IMU sensors (accelerometer, gyroscope, compass)
- Simulink-based control interface

## Controllers

### simulink_control_app
Main Simulink integration controller for advanced control algorithms.

### my_controller
Python-based keyboard controller for manual operation:
- **W**: Move forward
- **S**: Move backward
- **A**: Turn left
- **D**: Turn right

## Quick Start

1. Open `worlds/world.wbt` in Webots
2. Set controller to `simulink_control_app` for Simulink control
3. Open `simulink_control.slx` in MATLAB
4. Run simulation

## File Structure

```
scout_v2.0/
├── controllers/
│   ├── my_controller/           # Keyboard control
│   └── simulink_control_app/    # Simulink integration
├── protos/
│   └── ScoutV2.proto           # Robot model definition
└── worlds/
    └── world.wbt               # Webots world file
```

See [full documentation](../../docs/examples/scout-v2.md) for detailed information.
