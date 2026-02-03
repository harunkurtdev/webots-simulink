# Webots-Simulink Bridge Documentation

Welcome to the documentation for the **Webots-Simulink Bridge**, a framework that enables communication between the Webots robotic simulator and Simulink. This bridge allows users to simulate and control robotic systems using Simulink while visualizing their behavior in Webots.

---

## Overview

The Webots-Simulink Bridge establishes a bidirectional communication channel between the Webots physics simulator and MATLAB/Simulink control environment. This integration enables:

| Capability | Description |
|------------|-------------|
| **Physics Simulation** | Webots provides realistic 3D dynamics, collision detection, and sensor modeling |
| **Control Design** | Simulink enables block-diagram based controller development with state-space, PID, and advanced control methods |
| **Real-Time Data Exchange** | Sensor readings (IMU, LiDAR, GPS, encoders) flow to Simulink; motor commands return to Webots |
| **ROS 2 Deployment** | Export validated Simulink controllers as ROS 2 packages for physical robot deployment |

---

## Available Examples

### Control Systems

=== "Inverted Pendulum"
    ![InvertedPendulumVideo](./assets/videos/inverted_pendulum/inverted_pendulum.gif)

    Classic control theory example demonstrating stabilization of an inherently unstable system using LQR control.

    [📖 Documentation](examples/inverted-pendulum.md)

=== "Rotary Inverted Pendulum"
    ![RotaryInvertedPendulumVideo](./assets/videos/rotary_inverted_pendulum/video1.gif)

    Advanced control system with rotary arm and pendulum dynamics, featuring swing-up and balancing control.

    [📖 Documentation](examples/rotary-inverted-pendulum.md)

### Aerial Vehicles

=== "Crazyflie Drone"
    ![Crazyflie](./assets/videos/crazyflie/video2.gif)

    Nano quadcopter simulation with full 6-DOF control, cascaded PID architecture, and sensor integration.

    [📖 Documentation](examples/crazyflie-drone.md)

=== "Mavic 2 Pro"
    ![Mavic2Pro](./assets/videos/mavic/video1.gif)

    Professional quadcopter drone with advanced flight control and GPS navigation capabilities.

    [📖 Documentation](examples/mavic-2-pro.md)

=== "Blimp (Airship)"
    Lighter-than-air vehicle simulation with buoyancy control and 3D navigation.

    [📖 Documentation](examples/blimp.md)

### Ground Vehicles

=== "TurtleBot3"
    ![TurtleBot3](./assets/images/turtlebot3/image.png)

    Popular educational mobile robot platform with differential drive kinematics and SLAM capabilities.

    [📖 Documentation](examples/turtlebot3.md)

=== "Scout V2.0"
    ![ScoutV2](./assets/images/scout_v2/scout_world.jpg)

    Four-wheel drive mobile robot for autonomous navigation research with skid-steer kinematics.

    [📖 Documentation](examples/scout-v2.md)

=== "Wheel Chair"
    ![WheelChair](./assets/videos/wheel-chair/video1.gif)

    Assistive mobility platform simulation with obstacle avoidance and safety features.

    [📖 Documentation](examples/wheel-chair.md)

=== "Tractor (Boomer)"
    Agricultural vehicle simulation with Ackermann steering geometry and precision agriculture applications.

    [📖 Documentation](examples/tractor.md)

=== "Tesla Model 3"
    Autonomous vehicle simulation with ADAS features, lane keeping, and adaptive cruise control.

    [📖 Documentation](examples/tesla-model3.md)

### Marine Vehicles

=== "BlueBoat USV"
    ![Blueboat](./assets/videos/blueboat/video3.gif)

    Unmanned surface vehicle for marine robotics with differential thrust control and GPS navigation.

    [📖 Documentation](examples/blueboat-usv.md)

=== "Pirana USV"
    ![PiranaUSV](./assets/videos/pirana/video1.gif)

    Military-grade unmanned surface vehicle with steerable propulsion system.

    [📖 Documentation](examples/pirana-usv.md)

### Industrial Robots

=== "UR5e Robot Arm"
    6-DOF collaborative robot arm with forward/inverse kinematics and trajectory planning.

    [📖 Documentation](examples/ur5e-arm.md)

### Humanoid Robots

=== "NAO Humanoid"
    Bipedal humanoid robot with ZMP-based walking control and full-body motion planning.

    [📖 Documentation](examples/nao-humanoid.md)

### Legged Robots

=== "Spot Quadruped"
    Boston Dynamics-style quadruped robot with MPC-based locomotion control.

    [📖 Documentation](examples/spot-quadruped.md)

### Swarm Robotics

=== "E-puck Swarm"
    Multi-robot swarm simulation with Reynolds flocking rules and collective behavior.

    [📖 Documentation](examples/epuck-swarm.md)

### Educational Robots

=== "Khepera IV"
    Compact differential drive robot for research and education with extensive sensor suite.

    [📖 Documentation](examples/khepera-iv.md)

=== "Thymio II"
    Educational robot with reactive control architecture and visual programming support.

    [📖 Documentation](examples/thymio-ii.md)

=== "Firebird 6"
    Multi-sensor educational robot platform with EKF-based localization.

    [📖 Documentation](examples/firebird-6.md)

---

## Key Features

| Feature | Description |
|---------|-------------|
| **19 Robot Examples** | Pre-configured models for drones, mobile robots, marine vehicles, manipulators, and control systems |
| **State-Space Modeling** | Each example includes linearized A, B, C, D matrices for controller design |
| **Cascaded Control** | Hierarchical control architectures with position, velocity, and attitude loops |
| **Sensor Fusion** | Integration of LiDAR, IMU, GPS, encoders, and cameras with Extended Kalman Filter examples |
| **ROS 2 Compatibility** | Export Simulink models as ROS 2 nodes using MATLAB Coder |

---

## Quick Start

Follow these four steps to begin using the Webots-Simulink Bridge:

### Step 1: Install Prerequisites
Ensure that MATLAB/Simulink (R2020b or later) and Webots (R2023a or later) are installed on your system. Both applications must be properly licensed and configured. See the [Requirements](installation/requirements.md) page for version compatibility and system specifications.

### Step 2: Configure Your Environment
Set up the required environment variables and add the bridge files to your MATLAB path. The [Setup Guide](installation/setup.md) provides platform-specific instructions for Windows, macOS, and Linux.

### Step 3: Establish the Connection
Open your Webots world file and configure MATLAB as the external controller. The [Connecting Guide](usage/connecting.md) explains how to initialize the TCP/IP communication between Webots and Simulink.

### Step 4: Run a Simulation
Load a Simulink model and start the Webots simulation. Sensor data flows from Webots to Simulink, and control commands return to Webots in real-time. The [Running Simulations](usage/running.md) guide provides detailed instructions for each example project.

---

## Project Architecture

```
webots-simulink/
├── examples/                        # Example projects
│   ├── inverted_pendulum/          # Control system examples
│   ├── rotary_inverted_pendulum/
│   ├── crazyflie/                  # Aerial vehicles
│   ├── mavic_2_pro/
│   ├── blimp/
│   ├── turtlebot3/                 # Ground vehicles
│   ├── scout_v2.0/
│   ├── wheel_chair/
│   ├── tractor/
│   ├── tesla_model3/
│   ├── blueboat_usv/               # Marine vehicles
│   ├── pirana_usv/
│   ├── ur5e/                       # Industrial robots
│   ├── nao/                        # Humanoid robots
│   ├── spot/                       # Legged robots
│   ├── epuck_swarm/                # Swarm robotics
│   ├── khepera_iv/                 # Educational robots
│   ├── thymio_ii/
│   └── firebird_6/
├── docs/                            # Documentation
└── mkdocs.yml                       # Documentation config
```

---

## Documentation Structure

### Installation

| Page | Description |
|------|-------------|
| [Requirements](installation/requirements.md) | Software versions, operating system support, and hardware specifications |
| [Setup](installation/setup.md) | Environment configuration for Windows, macOS, and Linux |

### Usage

| Page | Description |
|------|-------------|
| [Connecting](usage/connecting.md) | TCP/IP communication setup between Webots and Simulink |
| [Running](usage/running.md) | Starting simulations and monitoring data exchange |
| [Customization](usage/customization.md) | Modifying the bridge for custom robot models |

### Advanced Topics

| Page | Description |
|------|-------------|
| [Debugging](advanced/debugging.md) | Diagnosing communication errors and timing issues |
| [ROS 2 Export](advanced/export-ros2.md) | Converting Simulink models to ROS 2 nodes for hardware deployment |

### Reference

| Page | Description |
|------|-------------|
| [Troubleshooting](troubleshooting.md) | Common errors and their solutions |
| [FAQ](faq.md) | Frequently asked questions |
| [Contributing](contributing/adding-new-examples.md) | Guidelines for adding new robot examples |

---

## Citation

If you use this work in your research, please cite:

> Kurt, H., Cayir, A., & Erkan, K. (2025). *Simulation Based Control Architecture Using Webots and Simulink*. arXiv. https://doi.org/10.48550/ARXIV.2505.02081
