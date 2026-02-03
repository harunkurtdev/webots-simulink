# Frequently Asked Questions (FAQ)

This page addresses common questions about the Webots-Simulink integration framework.

---

## General Questions

### What is Webots-Simulink Bridge?

The Webots-Simulink Bridge is an integration framework that allows you to use MATLAB/Simulink for designing and simulating control systems while Webots provides realistic 3D physics simulation. This combination enables rapid prototyping and testing of control algorithms in a virtual environment before deploying to real hardware.

### What are the system requirements?

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **Operating System** | Windows 10, macOS 10.15, Ubuntu 20.04 | Windows 11, macOS 13+, Ubuntu 22.04 |
| **MATLAB** | R2020a | R2022b or later |
| **Simulink** | Included with MATLAB | Latest version |
| **Webots** | R2023a | R2023b or later |
| **RAM** | 8 GB | 16 GB |
| **GPU** | OpenGL 3.3 compatible | Dedicated GPU with 4GB+ VRAM |

### Which robot platforms are supported?

Currently, the framework includes examples for:

- **Control Systems**: Inverted Pendulum, Rotary Inverted Pendulum
- **Aerial Vehicles**: Crazyflie Drone (Quadrotor)
- **Ground Vehicles**: TurtleBot3, Scout V2.0, Wheel Chair, Tractor (Boomer)
- **Marine Vehicles**: BlueBoat USV, Pirana USV

---

## Installation & Setup

### How do I install the framework?

1. Install MATLAB with Simulink
2. Install Webots (R2023a or later)
3. Clone this repository
4. Configure MATLAB path to include the project folders
5. Open Webots and load a world file from the `examples/` directory

See the [Installation Guide](installation/setup.md) for detailed instructions.

### Why can't MATLAB find Webots functions?

Ensure that:

1. Webots is properly installed and added to your system PATH
2. MATLAB has the Webots library path configured
3. You're running MATLAB from within Webots (using the MATLAB controller)

```matlab
% Add Webots MATLAB library to path (example for Windows)
addpath('C:\Program Files\Webots\lib\controller\matlab');
```

### How do I configure the TIME_STEP?

The `TIME_STEP` variable in the MATLAB controller should match the `basicTimeStep` in your Webots world file. A typical value is 16 ms (equivalent to ~62.5 Hz simulation rate).

```matlab
TIME_STEP = 16;  % milliseconds
```

---

## Simulink Integration

### How does Simulink communicate with Webots?

The framework uses MATLAB External Mode to create a real-time link between Simulink models and Webots. Key components:

1. **Wrapper Functions**: `wb_*.m` files that interface with Webots API
2. **Simulink Model**: Contains control logic and state-space representations
3. **MATLAB Script**: Initializes sensors/actuators and loads the Simulink model

### What are the wrapper functions?

Wrapper functions provide a MATLAB interface to Webots devices:

| Function | Purpose |
|----------|---------|
| `wb_motor_set_velocity.m` | Set motor angular velocity |
| `wb_motor_set_position.m` | Set motor target position |
| `wb_motor_set_torque.m` | Set motor torque |
| `wb_gyro_get_values.m` | Read gyroscope data |
| `wb_accelerometer_get_values.m` | Read accelerometer data |
| `wb_gps_get_values.m` | Read GPS coordinates |
| `wb_inertial_unit_get_roll_pitch_yaw.m` | Read orientation |
| `wb_lidar_get_range_image.m` | Read LiDAR scan data |
| `wb_robot_step.m` | Advance simulation by one time step |

### Can I use Stateflow with this framework?

Yes, Stateflow can be integrated into the Simulink models for implementing state machines and supervisory control logic. Simply add Stateflow charts to your control model.

---

## Control System Design

### How do I implement PID control?

Most examples include a `simulink_control.slx` model where you can add PID controllers:

1. Open the Simulink model
2. Add a PID Controller block from the Simulink library
3. Connect sensor inputs to the PID block
4. Tune the PID gains using MATLAB's PID Tuner app

### What is the state-space model used for?

The `state_space_modeling.slx` file in each example contains the linearized system model. This is useful for:

- Controller design (LQR, pole placement)
- System analysis (stability, controllability)
- Observer/estimator design
- Model-based control strategies

### How do I design an LQR controller?

```matlab
% Example for inverted pendulum
A = [...];  % State matrix
B = [...];  % Input matrix
Q = diag([10, 1, 100, 1]);  % State weight matrix
R = 1;  % Input weight

K = lqr(A, B, Q, R);  % Compute LQR gain
```

---

## Sensors & Actuators

### How do I read sensor data?

Example for reading IMU data:

```matlab
% Initialize sensor
imu = wb_robot_get_device('inertial_unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% Read values (in main loop)
orientation = wb_inertial_unit_get_roll_pitch_yaw(imu);
roll = orientation(1);
pitch = orientation(2);
yaw = orientation(3);
```

### How do I control motors?

```matlab
% Initialize motor
motor = wb_robot_get_device('motor_name');

% Velocity control
wb_motor_set_velocity(motor, 10.0);  % rad/s

% Position control
wb_motor_set_position(motor, 1.57);  % radians

% Torque control
wb_motor_set_torque(motor, 0.5);  % N·m
```

### What sensor filtering should I use?

The examples use low-pass filtering for noisy sensor data:

```matlab
% Low-pass filter coefficients
alpha_pitch = 0.85;
alpha_roll = 0.85;
alpha_yaw = 0.85;

% Apply filter
filtered_pitch = alpha_pitch * new_pitch + (1 - alpha_pitch) * old_pitch;
```

---

## Troubleshooting

### The simulation runs too slowly

- Increase `basicTimeStep` in the world file
- Reduce physics complexity (fewer collision shapes)
- Disable unnecessary sensors
- Use simpler 3D models

### Simulink model doesn't load

Check that:
1. The model file exists in the controller directory
2. MATLAB path includes the controller folder
3. No syntax errors in the initialization script

### Motors don't respond

Verify:
1. Motor names match between Webots and MATLAB
2. Motors are properly initialized with `wb_robot_get_device()`
3. Correct control mode is set (velocity, position, or torque)

---

## Contributing

### How can I add a new robot example?

See the [Adding New Examples](contributing/adding-new-examples.md) guide for detailed instructions on:

- Project structure requirements
- Wrapper function templates
- Documentation standards
- Testing procedures

### Where can I report issues?

Please report issues on the [GitHub repository](https://github.com/harunkurtdev/webots-simulink/issues).

---

## Additional Resources

- [Webots User Guide](https://cyberbotics.com/doc/guide/index)
- [Webots MATLAB API Reference](https://cyberbotics.com/doc/guide/matlab)
- [MathWorks Simulink Documentation](https://www.mathworks.com/help/simulink/)
- [Control System Toolbox](https://www.mathworks.com/help/control/)
