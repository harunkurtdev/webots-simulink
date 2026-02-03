# Blimp (Airship) Robot

This example demonstrates lighter-than-air vehicle (blimp/airship) control using MATLAB/Simulink with Webots simulation. Blimps offer unique advantages for aerial surveillance, communication relay, and research due to their energy efficiency and long endurance.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Lighter-Than-Air Vehicle (LTA) |
| **Difficulty** | Intermediate |
| **Control Method** | 6-DOF Flight Control / PID |
| **Propulsion** | Vectored thrust (gondola motors) |
| **Sensors** | IMU, GPS, Camera, Altimeter |

---

## Features

- **Buoyancy-Assisted Flight**: Helium-filled envelope
- **Vectored Thrust**: Tilting propellers for maneuverability
- **Low Energy Consumption**: Efficient hovering
- **6-DOF Control**: Position and attitude control
- **Camera Payload**: Surveillance/monitoring capability
- **Wind Resistance**: Disturbance rejection

---

## Project Structure

```
blimp/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m        # Main MATLAB controller
│       ├── simulink_control.slx          # Simulink control model
│       ├── state_space_modeling.slx      # State-space model
│       ├── wb_motor_set_velocity.m       # Motor velocity control
│       ├── wb_motor_set_position.m       # Motor/fin position control
│       ├── wb_gyro_get_values.m          # Gyroscope reading
│       ├── wb_accelerometer_get_values.m # Accelerometer reading
│       ├── wb_gps_get_values.m           # GPS position
│       ├── wb_inertial_unit_get_roll_pitch_yaw.m
│       ├── wb_distance_sensor_get_value.m # Altimeter
│       └── wb_robot_step.m               # Simulation step
└── worlds/
    └── blimp_outdoor.wbt                 # Webots world file
```

---

## Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Envelope Length | ~3 m |
| Envelope Diameter | ~1 m |
| Gondola Weight | ~2 kg |
| Buoyancy Gas | Helium |
| Max Speed | ~5 m/s |
| Endurance | Hours (depending on battery) |

### Propulsion System

| Motor | Webots Name | Function |
|-------|-------------|----------|
| Left Propeller | `left_motor` | Thrust + yaw |
| Right Propeller | `right_motor` | Thrust + yaw |
| Tail Propeller | `tail_motor` | Pitch control |
| Elevator | `elevator_motor` | Pitch trim |
| Rudder | `rudder_motor` | Yaw trim |

### Sensors

| Sensor | Webots Name | Purpose |
|--------|-------------|---------|
| Accelerometer | `accelerometer` | Linear acceleration |
| Gyroscope | `gyro` | Angular velocity |
| GPS | `gps` | Global position |
| Inertial Unit | `inertial_unit` | Orientation (RPY) |
| Altimeter | `altimeter` | Height above ground |
| Camera | `camera` | Visual input |

---

## Flight Dynamics

### Forces Acting on Blimp

1. **Buoyancy Force**: Upward force from helium
   - $F_b = \rho_{air} \cdot V \cdot g$

2. **Gravity Force**: Downward force
   - $F_g = m \cdot g$

3. **Thrust Force**: From propellers
   - Vectored for 3D control

4. **Drag Force**: Air resistance
   - Significant due to large envelope

### Simplified Dynamics

```matlab
% Blimp parameters
m = 2.0;           % Total mass (kg)
V_envelope = 1.5;  % Envelope volume (m³)
rho_air = 1.225;   % Air density (kg/m³)
g = 9.81;          % Gravity (m/s²)

% Buoyancy
F_buoyancy = rho_air * V_envelope * g;

% Net vertical force
F_net_z = F_buoyancy - m * g + thrust_z;

% Equations of motion
x_ddot = (thrust_x - drag_x) / m;
y_ddot = (thrust_y - drag_y) / m;
z_ddot = (F_buoyancy - m*g + thrust_z - drag_z) / m;
```

---

## Control Architecture

### Altitude Control

```matlab
function thrust_z = altitude_control(z_ref, z, z_dot)
    % PID altitude controller
    Kp_z = 5.0;
    Kd_z = 2.0;
    Ki_z = 0.5;

    e_z = z_ref - z;
    e_z_dot = -z_dot;

    thrust_z = Kp_z * e_z + Kd_z * e_z_dot + Ki_z * integral_z;

    % Account for buoyancy (may need less thrust to hover)
    thrust_z = thrust_z + gravity_compensation;
end
```

### Heading Control

```matlab
function [left_thrust, right_thrust] = heading_control(yaw_ref, yaw, yaw_dot)
    % Differential thrust for yaw control
    Kp_yaw = 2.0;
    Kd_yaw = 0.5;

    e_yaw = yaw_ref - yaw;
    e_yaw = atan2(sin(e_yaw), cos(e_yaw));  % Normalize

    yaw_cmd = Kp_yaw * e_yaw + Kd_yaw * (-yaw_dot);

    base_thrust = hover_thrust;
    left_thrust = base_thrust + yaw_cmd;
    right_thrust = base_thrust - yaw_cmd;
end
```

### Position Control

```matlab
function [thrust_cmd, heading_cmd] = position_control(pos_ref, pos, vel)
    % Horizontal position control
    Kp_xy = 1.0;
    Kd_xy = 0.5;

    e_x = pos_ref(1) - pos(1);
    e_y = pos_ref(2) - pos(2);

    vx_cmd = Kp_xy * e_x - Kd_xy * vel(1);
    vy_cmd = Kp_xy * e_y - Kd_xy * vel(2);

    % Convert to thrust magnitude and heading
    thrust_cmd = sqrt(vx_cmd^2 + vy_cmd^2);
    heading_cmd = atan2(vy_cmd, vx_cmd);
end
```

---

## Controller Implementation

### Initialization

```matlab
% simulink_control_app.m
TIME_STEP = 16;

% Physical parameters
mass = 2.0;  % kg
buoyancy_force = 1.225 * 1.5 * 9.81;  % N

% Initialize sensors
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

inertial_unit = wb_robot_get_device('inertial_unit');
wb_inertial_unit_enable(inertial_unit, TIME_STEP);

altimeter = wb_robot_get_device('altimeter');
wb_distance_sensor_enable(altimeter, TIME_STEP);

% Initialize motors
left_motor = wb_robot_get_device('left_motor');
right_motor = wb_robot_get_device('right_motor');
tail_motor = wb_robot_get_device('tail_motor');

wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_position(tail_motor, inf);

% Initialize control surfaces
elevator = wb_robot_get_device('elevator_motor');
rudder = wb_robot_get_device('rudder_motor');

% Initialize camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP * 4);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
```

---

## Flight Modes

### 1. Takeoff / Ascend

```matlab
function motors = takeoff(current_alt, target_alt)
    if current_alt < target_alt
        % Increase thrust above neutral buoyancy
        thrust = neutral_thrust * 1.3;
    else
        thrust = neutral_thrust;
    end
    motors = [thrust, thrust, 0];  % Left, right, tail
end
```

### 2. Hover / Station-Keeping

```matlab
function motors = hover(pos_ref, current_pos, current_att)
    % Maintain position against wind
    [thrust, heading] = position_control(pos_ref, current_pos);
    [left, right] = heading_control(heading, current_att(3));

    altitude_thrust = altitude_control(pos_ref(3), current_pos(3));

    motors = [left + altitude_thrust/2, right + altitude_thrust/2, 0];
end
```

### 3. Cruise / Forward Flight

```matlab
function [motors, surfaces] = cruise(waypoint, current_state)
    % Navigate to waypoint
    heading_ref = atan2(waypoint(2) - current_state.y, ...
                        waypoint(1) - current_state.x);

    [left, right] = heading_control(heading_ref, current_state.yaw);

    % Forward thrust
    cruise_thrust = 0.5 * max_thrust;

    motors = [cruise_thrust + left, cruise_thrust + right, 0];
    surfaces = [0, 0];  % Elevator, rudder trim
end
```

### 4. Landing / Descend

```matlab
function motors = landing(current_alt)
    if current_alt > 0.5
        % Reduce thrust below neutral
        thrust = neutral_thrust * 0.7;
    else
        thrust = 0;  % Motors off
    end
    motors = [thrust, thrust, 0];
end
```

---

## Quick Start

1. **Open Webots** and load `examples/blimp/worlds/blimp_outdoor.wbt`

2. **Configure MATLAB** and set Webots library path

3. **Run the simulation**:
   - Blimp will ascend to target altitude
   - Hover and maintain position
   - Follow waypoints if programmed

4. **Customize flight**:
   - Modify target altitude and waypoints
   - Tune PID gains for wind conditions
   - Add surveillance patterns

---

## Challenges and Considerations

### Wind Disturbance

Blimps are sensitive to wind due to large surface area:
- Implement robust disturbance rejection
- Use GPS velocity for wind estimation
- Consider feedforward compensation

### Slow Dynamics

Blimps have slow response compared to multirotors:
- Use appropriate controller gains
- Plan smooth trajectories
- Account for momentum in maneuvers

### Buoyancy Changes

Gas temperature affects buoyancy:
- Monitor altitude drift
- Adjust thrust bias as needed
- Consider day/night variations

---

## Applications

- **Surveillance**: Long-duration observation
- **Communication Relay**: Aerial platform
- **Advertising**: Large display surface
- **Research**: Atmospheric monitoring
- **Agriculture**: Crop monitoring

---

## References

- [Webots Blimp Tutorial](https://cyberbotics.com/doc/guide/blimp)
- Elfes, A. et al. (1998). "Autonomous Robotic Airships for Planetary Exploration"
- Mueller, J. (2014). "Design and Control of an Indoor Micro Quadrotor"
- Hygounenc, E. et al. (2004). "The autonomous blimp project"
