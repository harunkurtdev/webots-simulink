# Webots-Simulink Bridge Documentation

Welcome to the documentation for the **Webots-Simulink Bridge**, a framework that enables communication between the Webots robotic simulator and Simulink. This bridge allows users to simulate and control robotic systems using Simulink while visualizing their behavior in Webots.

---

## Overview

This project enables real-time interaction between Simulink and Webots allowing:

- **Simulation of robotic systems** using Webots' physics engine
- **Control algorithms** to be implemented in Simulink
- **Direct function calls** from MATLAB to Webots for data exchange
- **ROS 2 integration** for advanced robotic applications

---

## Available Examples

### Control Systems

=== "Inverted Pendulum"
    ![InvertedPendulumVideo](./assets/videos/inverted_pendulum/inverted_pendulum.gif)

    Classic control theory example demonstrating stabilization of an inherently unstable system.

=== "Rotary Inverted Pendulum"
    ![RotaryInvertedPendulumVideo](./assets/videos/rotary_inverted_pendulum/video1.gif)

    Advanced control system with rotary arm and pendulum dynamics.

### Aerial Vehicles

=== "Crazyflie Drone"
    ![Crazyflie](./assets/videos/crazyflie/video2.gif)

    Nano quadcopter simulation with full 6-DOF control and sensor integration.

### Ground Vehicles

=== "TurtleBot3"
    ![TurtleBot3](./assets/images/turtlebot3/image.png)

    Popular educational mobile robot platform with differential drive kinematics.

=== "Scout V2.0"
    ![ScoutV2](./assets/images/scout_v2/scout_world.jpg)

    Four-wheel drive mobile robot for autonomous navigation research.

=== "Wheel Chair"
    ![WheelChair](./assets/videos/wheel-chair/video1.gif)

    Assistive mobility platform simulation.

=== "Tractor (Boomer)"
    Agricultural vehicle simulation with steering and drive control.

### Marine Vehicles

=== "BlueBoat USV"
    ![Blueboat](./assets/videos/blueboat/video3.gif)

    Unmanned surface vehicle for marine robotics applications.

=== "Pirana USV"
    ![PiranaUSV](./assets/videos/pirana/video1.gif)

    Military-grade unmanned surface vehicle platform.

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Multi-Platform Support** | Ground vehicles, aerial robots, marine vessels, and control systems |
| **Real-Time Data Exchange** | Bidirectional communication between Webots and Simulink |
| **Customizable Bridge** | Adapt the implementation for different use cases |
| **ROS 2 Integration** | Export Simulink models to ROS 2 packages |
| **Sensor Integration** | LiDAR, IMU, GPS, cameras, and more |

---

## Quick Start

1. **Install Prerequisites**: Check the [Requirements](installation/requirements.md) page
2. **Setup Environment**: Follow the [Setup Guide](installation/setup.md)
3. **Connect Webots & Simulink**: Read [Connecting Guide](usage/connecting.md)
4. **Run Your First Simulation**: See [Running Simulations](usage/running.md)

---

## Project Architecture

```
webots-simulink/
├── examples/                    # Example projects
│   ├── inverted_pendulum/      # Control system examples
│   ├── rotary_inverted_pendulum/
│   ├── crazyflie/              # Aerial vehicle
│   ├── turtlebot3/             # Ground vehicles
│   ├── scout_v2.0/
│   ├── wheel_chair/
│   ├── tractor/
│   ├── blueboat_usv/           # Marine vehicles
│   └── pirana_usv/
├── docs/                        # Documentation
└── mkdocs.yml                   # Documentation config
```

---

## Documentation Structure

- **[Installation](installation/requirements.md)**: System requirements and setup instructions
- **[Usage](usage/connecting.md)**: Connecting Webots and Simulink, running simulations
- **[Advanced Topics](advanced/debugging.md)**: Debugging, performance tuning, ROS 2 export
- **[Examples](examples/inverted-pendulum.md)**: Detailed documentation for each example
- **[Troubleshooting](troubleshooting.md)**: Common issues and solutions
- **[FAQ](faq.md)**: Frequently asked questions

---

## Citation

If you use this work in your research, please cite:

> Kurt, H., Cayir, A., & Erkan, K. (2025). *Simulation Based Control Architecture Using Webots and Simulink*. arXiv. https://doi.org/10.48550/ARXIV.2505.02081
