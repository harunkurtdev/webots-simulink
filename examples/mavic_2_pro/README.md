# Mavic 2 Pro Drone

This example demonstrates quadrotor drone control using the DJI Mavic 2 Pro model with MATLAB/Simulink. The Mavic 2 Pro is a professional-grade drone with advanced camera and sensing capabilities.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Quadrotor UAV |
| **Robot** | DJI Mavic 2 Pro |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | 6-DOF Flight Control / PID |
| **Sensors** | Camera, GPS, IMU, Range sensors |

---

## Features

- **6-DOF Flight Control**: Full attitude and position control
- **GPS Navigation**: Waypoint following and position hold
- **Obstacle Avoidance**: Front/bottom range sensors
- **Camera Gimbal**: 2-axis stabilized camera
- **Autonomous Flight**: Takeoff, landing, waypoint missions
- **High-fidelity Dynamics**: Realistic aerodynamic model

---

## Project Structure

```
mavic_2_pro/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m        # Main MATLAB controller
│       ├── simulink_control.slx          # Simulink control model
│       ├── state_space_modeling.slx      # State-space model
│       ├── wb_motor_set_velocity.m       # Motor velocity control
│       ├── wb_gyro_get_values.m          # Gyroscope reading
│       ├── wb_accelerometer_get_values.m # Accelerometer reading
│       ├── wb_gps_get_values.m           # GPS position
│       ├── wb_inertial_unit_get_roll_pitch_yaw.m
│       ├── wb_camera_get_image.m         # Camera image
│       ├── wb_distance_sensor_get_value.m # Range sensors
│       └── wb_robot_step.m               # Simulation step
└── worlds/
    └── mavic_flight.wbt                  # Webots world file
```

---

## Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Diagonal Size | 354 mm |
| Weight | 907 g |
| Max Flight Time | ~31 min |
| Max Speed | 20 m/s (S-mode) |
| Max Ascent Speed | 5 m/s |
| Max Descent Speed | 3 m/s |

### Motors

| Motor | Webots Name | Position |
|-------|-------------|----------|
| Front Right | `front right propeller` | CW |
| Front Left | `front left propeller` | CCW |
| Rear Right | `rear right propeller` | CCW |
| Rear Left | `rear left propeller` | CW |

### Sensors

| Sensor | Webots Name | Purpose |
|--------|-------------|---------|
| Accelerometer | `accelerometer` | Linear acceleration |
| Gyroscope | `gyro` | Angular velocity |
| GPS | `gps` | Global position |
| Inertial Unit | `inertial unit` | Orientation |
| Front Camera | `camera` | Visual input |
| Bottom Range | `bottom range sensor` | Altitude |
| Front Range | `front range sensor` | Obstacle detection |

### Camera Gimbal

| Actuator | Webots Name | Range |
|----------|-------------|-------|
| Pitch Motor | `camera pitch` | -90° to +30° |
| Roll Motor | `camera roll` | ±35° |

---

## Flight Dynamics

### Motor Mixing

```matlab
% Quadrotor motor mixing for X configuration
%   1 (FR)    2 (FL)
%      \      /
%       \    /
%        \  /
%         \/
%         /\
%        /  \
%       /    \
%      /      \
%   4 (RR)    3 (RL)

function [m1, m2, m3, m4] = motor_mixing(throttle, roll, pitch, yaw)
    % m1 = Front Right (CW)
    % m2 = Front Left (CCW)
    % m3 = Rear Left (CW)
    % m4 = Rear Right (CCW)

    m1 = throttle - roll + pitch - yaw;
    m2 = throttle + roll + pitch + yaw;
    m3 = throttle + roll - pitch - yaw;
    m4 = throttle - roll - pitch + yaw;

    % Clamp to valid range
    m1 = max(0, min(600, m1));
    m2 = max(0, min(600, m2));
    m3 = max(0, min(600, m3));
    m4 = max(0, min(600, m4));
end
```

### State-Space Model

```matlab
% Linearized quadrotor dynamics around hover
% State: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]

% Simplified altitude dynamics
A_z = [0 1; 0 0];
B_z = [0; 1/m];

% Roll dynamics
A_roll = [0 1; 0 0];
B_roll = [0; 1/Ixx];

% Pitch dynamics
A_pitch = [0 1; 0 0];
B_pitch = [0; 1/Iyy];

% Yaw dynamics
A_yaw = [0 1; 0 0];
B_yaw = [0; 1/Izz];
```

---

## Control Architecture

### Cascaded PID Control

```
                    ┌─────────────┐
Position Ref ──────>│  Position   │──────> Attitude Ref
                    │  Controller │
                    └─────────────┘
                           │
                           v
                    ┌─────────────┐
                    │  Attitude   │──────> Rate Ref
                    │  Controller │
                    └─────────────┘
                           │
                           v
                    ┌─────────────┐
                    │    Rate     │──────> Motor Commands
                    │  Controller │
                    └─────────────┘
```

### Position Controller

```matlab
function [roll_cmd, pitch_cmd, thrust] = position_control(pos_ref, pos, vel)
    % PID gains
    Kp_xy = 1.0; Kd_xy = 0.5;
    Kp_z = 2.0; Kd_z = 1.0;

    % Position errors
    ex = pos_ref(1) - pos(1);
    ey = pos_ref(2) - pos(2);
    ez = pos_ref(3) - pos(3);

    % Velocity errors (derivative)
    ex_dot = -vel(1);
    ey_dot = -vel(2);
    ez_dot = -vel(3);

    % Desired accelerations
    ax_des = Kp_xy * ex + Kd_xy * ex_dot;
    ay_des = Kp_xy * ey + Kd_xy * ey_dot;
    az_des = Kp_z * ez + Kd_z * ez_dot + g;  % Gravity compensation

    % Convert to attitude commands
    roll_cmd = (ax_des * sin(yaw) - ay_des * cos(yaw)) / g;
    pitch_cmd = (ax_des * cos(yaw) + ay_des * sin(yaw)) / g;
    thrust = m * az_des;
end
```

### Attitude Controller

```matlab
function [p_cmd, q_cmd, r_cmd] = attitude_control(att_ref, att)
    % PID gains
    Kp_roll = 5.0; Kp_pitch = 5.0; Kp_yaw = 2.0;

    % Attitude errors
    e_roll = att_ref(1) - att(1);
    e_pitch = att_ref(2) - att(2);
    e_yaw = att_ref(3) - att(3);

    % Normalize yaw error
    e_yaw = atan2(sin(e_yaw), cos(e_yaw));

    % Rate commands
    p_cmd = Kp_roll * e_roll;
    q_cmd = Kp_pitch * e_pitch;
    r_cmd = Kp_yaw * e_yaw;
end
```

---

## Controller Implementation

### Initialization

```matlab
% simulink_control_app.m
TIME_STEP = 8;  % 125 Hz control rate

% Physical parameters
m = 0.907;  % kg
g = 9.81;   % m/s^2
Ixx = 0.01; Iyy = 0.01; Izz = 0.02;

% Initialize IMU
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% Initialize GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% Initialize range sensors
front_range = wb_robot_get_device('front range sensor');
bottom_range = wb_robot_get_device('bottom range sensor');
wb_distance_sensor_enable(front_range, TIME_STEP);
wb_distance_sensor_enable(bottom_range, TIME_STEP);

% Initialize motors
motors = cell(1, 4);
motor_names = {'front right propeller', 'front left propeller', ...
               'rear left propeller', 'rear right propeller'};
for i = 1:4
    motors{i} = wb_robot_get_device(motor_names{i});
    wb_motor_set_position(motors{i}, inf);
    wb_motor_set_velocity(motors{i}, 0);
end

% Initialize camera gimbal
camera_pitch = wb_robot_get_device('camera pitch');
camera_roll = wb_robot_get_device('camera roll');

% Initialize camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP * 4);  % 30 Hz

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
```

---

## Flight Modes

### 1. Takeoff

```matlab
function motors = takeoff(current_alt, target_alt)
    if current_alt < target_alt
        thrust = hover_thrust * 1.2;
    else
        thrust = hover_thrust;
    end
    motors = [thrust, thrust, thrust, thrust];
end
```

### 2. Hover / Position Hold

```matlab
function motors = hover(pos_ref, current_pos, current_att, current_vel)
    [roll_cmd, pitch_cmd, thrust] = position_control(pos_ref, current_pos, current_vel);
    [p_cmd, q_cmd, r_cmd] = attitude_control([roll_cmd, pitch_cmd, 0], current_att);
    motors = motor_mixing(thrust, p_cmd, q_cmd, r_cmd);
end
```

### 3. Waypoint Navigation

```matlab
function [pos_ref, yaw_ref] = waypoint_navigation(waypoints, current_pos, wp_index)
    if norm(waypoints(wp_index,:) - current_pos) < 0.5
        wp_index = min(wp_index + 1, size(waypoints, 1));
    end

    pos_ref = waypoints(wp_index, :);
    dx = pos_ref(1) - current_pos(1);
    dy = pos_ref(2) - current_pos(2);
    yaw_ref = atan2(dy, dx);
end
```

### 4. Landing

```matlab
function motors = landing(current_alt)
    if current_alt > 0.1
        thrust = hover_thrust * 0.8;
    else
        thrust = 0;  % Motors off
    end
    motors = [thrust, thrust, thrust, thrust];
end
```

---

## Quick Start

1. **Open Webots** and load `examples/mavic_2_pro/worlds/mavic_flight.wbt`

2. **Start MATLAB** and configure the Webots library path

3. **Run the simulation**:
   - Drone will takeoff automatically
   - Hover at 2m altitude
   - Execute waypoint mission (if configured)
   - Land on command

4. **Customize flight**:
   - Modify waypoints in Simulink model
   - Tune PID gains for different responses
   - Add obstacle avoidance logic

---

## Safety Features

- **Geofencing**: Limit flight area
- **Low Battery Return**: Automatic RTH
- **Obstacle Detection**: Front sensor avoidance
- **Failsafe**: Motor cutoff on signal loss

---

## Applications

- **Aerial Photography**: Stabilized camera platform
- **Mapping**: Autonomous survey missions
- **Inspection**: Infrastructure monitoring
- **Research**: Flight control algorithm development

---

## References

- [DJI Mavic 2 Pro Specifications](https://www.dji.com/mavic-2/specs)
- [Webots Mavic Documentation](https://cyberbotics.com/doc/guide/mavic-2-pro)
- Mahony, R. et al. (2012). "Multirotor aerial vehicles: Modeling, estimation, and control"
- Bouabdallah, S. (2007). "Design and control of quadrotors with application to autonomous flying"
