# UR5e Robot Arm

This example demonstrates industrial robot arm control using the Universal Robots UR5e collaborative robot with MATLAB/Simulink. The UR5e is a 6-DOF robotic manipulator widely used in manufacturing and research.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | 6-DOF Collaborative Robot Arm |
| **Robot** | Universal Robots UR5e |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | Joint Space / Cartesian / Trajectory |
| **Applications** | Pick-and-place, Assembly, Welding |

---

## Features

- **6 Degrees of Freedom**: Full spatial manipulation
- **Joint Position Control**: Individual joint angle control
- **Inverse Kinematics**: Cartesian position to joint angles
- **Trajectory Planning**: Smooth motion profiles
- **Force/Torque Sensing**: Contact detection capability
- **Safety Features**: Collaborative operation support

---

## Project Structure

```
ur5e_arm/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m        # Main MATLAB controller
│       ├── simulink_control.slx          # Simulink control model
│       ├── state_space_modeling.slx      # State-space model
│       ├── wb_motor_set_position.m       # Joint position control
│       ├── wb_motor_set_velocity.m       # Joint velocity control
│       ├── wb_position_sensor_get_value.m # Joint encoders
│       ├── forward_kinematics.m          # FK computation
│       ├── inverse_kinematics.m          # IK computation
│       └── wb_robot_step.m               # Simulation step
└── worlds/
    └── ur5e_workspace.wbt                # Webots world file
```

---

## Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Weight | 20.6 kg |
| Payload | 5 kg |
| Reach | 850 mm |
| Footprint | Ø 149 mm |
| Repeatability | ±0.03 mm |
| DOF | 6 |

### Joint Specifications

| Joint | Name | Range | Max Speed |
|-------|------|-------|-----------|
| J1 | Base | ±360° | 180°/s |
| J2 | Shoulder | ±360° | 180°/s |
| J3 | Elbow | ±360° | 180°/s |
| J4 | Wrist 1 | ±360° | 180°/s |
| J5 | Wrist 2 | ±360° | 180°/s |
| J6 | Wrist 3 | ±360° | 180°/s |

### Webots Device Names

| Joint | Motor Name | Sensor Name |
|-------|------------|-------------|
| J1 | `shoulder_pan_joint` | `shoulder_pan_joint_sensor` |
| J2 | `shoulder_lift_joint` | `shoulder_lift_joint_sensor` |
| J3 | `elbow_joint` | `elbow_joint_sensor` |
| J4 | `wrist_1_joint` | `wrist_1_joint_sensor` |
| J5 | `wrist_2_joint` | `wrist_2_joint_sensor` |
| J6 | `wrist_3_joint` | `wrist_3_joint_sensor` |

---

## Kinematics

### DH Parameters

| i | a_i (m) | d_i (m) | alpha_i (rad) | theta_i |
|---|---------|---------|---------------|---------|
| 1 | 0 | 0.1625 | π/2 | θ₁ |
| 2 | -0.425 | 0 | 0 | θ₂ |
| 3 | -0.3922 | 0 | 0 | θ₃ |
| 4 | 0 | 0.1333 | π/2 | θ₄ |
| 5 | 0 | 0.0997 | -π/2 | θ₅ |
| 6 | 0 | 0.0996 | 0 | θ₆ |

### Forward Kinematics

```matlab
function T = forward_kinematics(q)
    % DH parameters
    a = [0, -0.425, -0.3922, 0, 0, 0];
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    T = eye(4);
    for i = 1:6
        T_i = dh_transform(a(i), d(i), alpha(i), q(i));
        T = T * T_i;
    end
end

function T = dh_transform(a, d, alpha, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
```

### Inverse Kinematics

```matlab
function q = inverse_kinematics(T_target, q0)
    % Numerical IK using Newton-Raphson
    q = q0;
    max_iter = 100;
    tol = 1e-6;

    for iter = 1:max_iter
        T_current = forward_kinematics(q);

        % Position error
        e_pos = T_target(1:3, 4) - T_current(1:3, 4);

        % Orientation error (using rotation matrix)
        R_error = T_target(1:3, 1:3) * T_current(1:3, 1:3)';
        e_rot = rotation_to_axis_angle(R_error);

        e = [e_pos; e_rot];

        if norm(e) < tol
            break;
        end

        % Jacobian
        J = compute_jacobian(q);

        % Update joint angles
        dq = pinv(J) * e;
        q = q + dq';
    end
end
```

### Jacobian Computation

```matlab
function J = compute_jacobian(q)
    % Geometric Jacobian
    J = zeros(6, 6);
    T = eye(4);
    z = zeros(3, 6);
    p = zeros(3, 6);

    % DH parameters
    a = [0, -0.425, -0.3922, 0, 0, 0];
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    z(:,1) = [0; 0; 1];
    p(:,1) = [0; 0; 0];

    for i = 1:6
        T_i = dh_transform(a(i), d(i), alpha(i), q(i));
        T = T * T_i;
        if i < 6
            z(:,i+1) = T(1:3, 3);
            p(:,i+1) = T(1:3, 4);
        end
    end

    p_ee = T(1:3, 4);

    for i = 1:6
        J(1:3, i) = cross(z(:,i), p_ee - p(:,i));
        J(4:6, i) = z(:,i);
    end
end
```

---

## Trajectory Planning

### Point-to-Point Motion

```matlab
function q_traj = generate_trajectory(q_start, q_end, duration, dt)
    % Quintic polynomial trajectory
    t = 0:dt:duration;
    n = length(t);
    q_traj = zeros(n, 6);

    for j = 1:6
        q0 = q_start(j);
        qf = q_end(j);

        % Quintic polynomial coefficients
        a0 = q0;
        a1 = 0;
        a2 = 0;
        a3 = 10*(qf - q0) / duration^3;
        a4 = -15*(qf - q0) / duration^4;
        a5 = 6*(qf - q0) / duration^5;

        for i = 1:n
            ti = t(i);
            q_traj(i, j) = a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4 + a5*ti^5;
        end
    end
end
```

### Linear Cartesian Motion

```matlab
function q_traj = linear_cartesian_trajectory(p_start, p_end, R, duration, dt)
    t = 0:dt:duration;
    n = length(t);
    q_traj = zeros(n, 6);

    q_current = inverse_kinematics(pose_to_transform(p_start, R), zeros(1,6));

    for i = 1:n
        s = t(i) / duration;  % Normalized time [0, 1]

        % Linear interpolation
        p = p_start + s * (p_end - p_start);

        % Create target transform
        T_target = pose_to_transform(p, R);

        % Solve IK
        q_traj(i, :) = inverse_kinematics(T_target, q_current);
        q_current = q_traj(i, :);
    end
end
```

---

## Controller Implementation

### Initialization

```matlab
% simulink_control_app.m
TIME_STEP = 16;

% Joint names
joint_names = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', ...
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Initialize motors and sensors
motors = cell(1, 6);
sensors = cell(1, 6);

for i = 1:6
    motors{i} = wb_robot_get_device(joint_names{i});
    sensors{i} = wb_robot_get_device([joint_names{i} '_sensor']);
    wb_position_sensor_enable(sensors{i}, TIME_STEP);
end

% Home position (joint angles in radians)
home_position = [0, -pi/2, pi/2, -pi/2, -pi/2, 0];

% Move to home
for i = 1:6
    wb_motor_set_position(motors{i}, home_position(i));
end

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
```

### Joint Position Control

```matlab
function set_joint_positions(motors, q_target)
    for i = 1:6
        wb_motor_set_position(motors{i}, q_target(i));
    end
end
```

### Cartesian Position Control

```matlab
function move_to_cartesian(motors, sensors, p_target, R_target)
    % Get current joint positions
    q_current = zeros(1, 6);
    for i = 1:6
        q_current(i) = wb_position_sensor_get_value(sensors{i});
    end

    % Compute IK
    T_target = pose_to_transform(p_target, R_target);
    q_target = inverse_kinematics(T_target, q_current);

    % Set joint positions
    set_joint_positions(motors, q_target);
end
```

---

## Quick Start

1. **Open Webots** and load `examples/ur5e_arm/worlds/ur5e_workspace.wbt`

2. **Start MATLAB** and configure Webots library path

3. **Run simulation**:
   - Robot moves to home position
   - Execute pick-and-place demo
   - Follow programmed trajectory

4. **Customize motion**:
   - Modify target positions in Simulink
   - Create custom trajectories
   - Implement force control (with sensor)

---

## Applications

- **Pick and Place**: Object manipulation
- **Assembly**: Component insertion, fastening
- **Welding**: Continuous path following
- **Machine Tending**: CNC loading/unloading
- **Research**: Motion planning algorithms

---

## Safety Considerations

- **Speed Limits**: Enforce joint velocity limits
- **Workspace Limits**: Prevent singularities and collisions
- **Force Limits**: Monitor joint torques
- **Emergency Stop**: Immediate halt capability

---

## References

- [Universal Robots UR5e](https://www.universal-robots.com/products/ur5-robot/)
- [Webots UR5e Documentation](https://cyberbotics.com/doc/guide/ure)
- Craig, J. J. (2005). "Introduction to Robotics: Mechanics and Control"
- Siciliano, B. et al. (2009). "Robotics: Modelling, Planning and Control"
