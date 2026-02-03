# Mavic 2 Pro Drone

This example demonstrates quadrotor drone control using the DJI Mavic 2 Pro with MATLAB/Simulink.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Quadrotor UAV |
| **Robot** | DJI Mavic 2 Pro |
| **Difficulty** | Intermediate to Advanced |
| **Control Method** | 6-DOF Flight Control / PID |

---

## Features

- **6-DOF Flight Control**: Full attitude and position control
- **GPS Navigation**: Waypoint following and position hold
- **Obstacle Avoidance**: Front/bottom range sensors
- **Camera Gimbal**: 2-axis stabilized camera
- **Autonomous Flight**: Takeoff, landing, waypoint missions

---

## Specifications

| Property | Value |
|----------|-------|
| Diagonal Size | 354 mm |
| Weight | 907 g |
| Max Speed | 20 m/s |
| Motors | 4 (X configuration) |
| Sensors | IMU, GPS, Camera, Range sensors |

---

## Control Architecture

### Cascaded Control System

```mermaid
flowchart TB
    subgraph Reference["Reference Commands"]
        WP[/"Waypoint<br/>(x_d, y_d, z_d, psi_d)"/]
    end

    subgraph Outer["Outer Loop - Position Control"]
        PC["Position<br/>Controller<br/>PID"]
        YC["Yaw<br/>Controller<br/>PID"]
    end

    subgraph Middle["Middle Loop - Attitude Control"]
        AC["Attitude<br/>Controller<br/>PID"]
    end

    subgraph Inner["Inner Loop - Rate Control"]
        RC["Rate<br/>Controller<br/>PID"]
    end

    subgraph Mixer["Motor Mixer"]
        MX["X-Config<br/>Mixing Matrix"]
    end

    subgraph Motors["Quadrotor Motors"]
        M1["M1<br/>Front-Right"]
        M2["M2<br/>Back-Right"]
        M3["M3<br/>Back-Left"]
        M4["M4<br/>Front-Left"]
    end

    subgraph Sensors["Sensor Suite"]
        GPS["GPS"]
        IMU["IMU"]
        GYRO["Gyroscope"]
        RANGE["Range<br/>Sensors"]
    end

    subgraph Plant["Quadrotor Dynamics"]
        DYN["6-DOF<br/>Rigid Body"]
    end

    WP --> PC
    WP --> YC

    PC -->|"phi_d, theta_d"| AC
    YC -->|"psi_d"| AC
    PC -->|"T_d"| MX

    AC -->|"p_d, q_d, r_d"| RC

    RC -->|"tau_phi, tau_theta, tau_psi"| MX

    MX --> M1 & M2 & M3 & M4

    M1 & M2 & M3 & M4 --> DYN

    DYN --> GPS & IMU & GYRO & RANGE

    GPS -->|"x, y, z"| PC
    IMU -->|"phi, theta, psi"| AC
    GYRO -->|"p, q, r"| RC
    RANGE -->|"obstacle distance"| PC

    style Reference fill:#e1f5fe
    style Outer fill:#e8f5e9
    style Middle fill:#fff3e0
    style Inner fill:#fce4ec
    style Mixer fill:#f3e5f5
    style Motors fill:#ffcdd2
    style Sensors fill:#c8e6c9
    style Plant fill:#bbdefb
```

### State-Space Model

```mermaid
flowchart LR
    subgraph Inputs["Control Inputs u"]
        U1["T - Total Thrust"]
        U2["tau_phi - Roll Torque"]
        U3["tau_theta - Pitch Torque"]
        U4["tau_psi - Yaw Torque"]
    end

    subgraph StateSpace["State-Space Model<br/>x_dot = Ax + Bu + G"]
        A["A Matrix<br/>12x12"]
        B["B Matrix<br/>12x4"]
        G["Gravity<br/>Vector"]
    end

    subgraph States["State Vector x"]
        S1["x, y, z<br/>Position"]
        S2["phi, theta, psi<br/>Euler Angles"]
        S3["u, v, w<br/>Body Velocities"]
        S4["p, q, r<br/>Angular Rates"]
    end

    subgraph Outputs["Outputs y"]
        Y1["Position - GPS"]
        Y2["Attitude - IMU"]
        Y3["Altitude - Barometer"]
    end

    U1 & U2 & U3 & U4 --> B
    B --> StateSpace
    A --> StateSpace
    G --> StateSpace
    StateSpace --> S1 & S2 & S3 & S4
    S1 --> Y1 & Y3
    S2 --> Y2

    style Inputs fill:#ffcdd2
    style StateSpace fill:#c8e6c9
    style States fill:#bbdefb
    style Outputs fill:#fff9c4
```

### Motor Mixing (X-Configuration)

```mermaid
flowchart TB
    subgraph ControlInputs["Control Commands"]
        T["Thrust T"]
        TPHI["Roll tau_phi"]
        TTHETA["Pitch tau_theta"]
        TPSI["Yaw tau_psi"]
    end

    subgraph MixingMatrix["X-Config Mixing"]
        MIX["Mixing<br/>Matrix M"]
    end

    subgraph MotorOutputs["Motor Speeds"]
        W1["omega_1 = T + tau_phi - tau_theta - tau_psi"]
        W2["omega_2 = T - tau_phi - tau_theta + tau_psi"]
        W3["omega_3 = T - tau_phi + tau_theta - tau_psi"]
        W4["omega_4 = T + tau_phi + tau_theta + tau_psi"]
    end

    subgraph Diagram["Motor Layout - Top View"]
        LAYOUT["<br/>    M4(CCW)   M1(CW)<br/>        \\     /<br/>         \\   /<br/>          \\ /<br/>           X<br/>          / \\<br/>         /   \\<br/>        /     \\<br/>    M3(CW)   M2(CCW)"]
    end

    T & TPHI & TTHETA & TPSI --> MIX
    MIX --> W1 & W2 & W3 & W4

    style ControlInputs fill:#e1f5fe
    style MixingMatrix fill:#c8e6c9
    style MotorOutputs fill:#fff9c4
    style Diagram fill:#f3e5f5
```

### Attitude Control Loop Detail

```mermaid
flowchart LR
    subgraph RollLoop["Roll Control"]
        PHI_D["phi_d"] --> E_PHI["Sigma"]
        PHI["phi"] --> E_PHI
        E_PHI --> PID_PHI["PID Roll"]
        PID_PHI --> P_D["p_d"]
        P_D --> E_P["Sigma"]
        P["p"] --> E_P
        E_P --> PID_P["PID Rate"]
        PID_P --> TAU_PHI["tau_phi"]
    end

    subgraph PitchLoop["Pitch Control"]
        THETA_D["theta_d"] --> E_THETA["Sigma"]
        THETA["theta"] --> E_THETA
        E_THETA --> PID_THETA["PID Pitch"]
        PID_THETA --> Q_D["q_d"]
        Q_D --> E_Q["Sigma"]
        Q["q"] --> E_Q
        E_Q --> PID_Q["PID Rate"]
        PID_Q --> TAU_THETA["tau_theta"]
    end

    subgraph YawLoop["Yaw Control"]
        PSI_D["psi_d"] --> E_PSI["Sigma"]
        PSI["psi"] --> E_PSI
        E_PSI --> PID_PSI["PID Yaw"]
        PID_PSI --> R_D["r_d"]
        R_D --> E_R["Sigma"]
        R["r"] --> E_R
        E_R --> PID_R["PID Rate"]
        PID_R --> TAU_PSI["tau_psi"]
    end

    style RollLoop fill:#ffcdd2
    style PitchLoop fill:#c8e6c9
    style YawLoop fill:#bbdefb
```

---

## State-Space Model

### 12-State Quadrotor Model

```matlab
% State vector: x = [x, y, z, phi, theta, psi, u, v, w, p, q, r]'
% Input vector: u = [T, tau_phi, tau_theta, tau_psi]'

% Physical parameters
m = 0.907;          % Mass [kg]
g = 9.81;           % Gravity [m/s^2]
L = 0.177;          % Arm length [m]
Ixx = 0.0029;       % Roll inertia [kg*m^2]
Iyy = 0.0029;       % Pitch inertia [kg*m^2]
Izz = 0.0055;       % Yaw inertia [kg*m^2]

% Linearized A matrix at hover (12x12)
A = zeros(12, 12);
A(1:3, 7:9) = eye(3);           % Position from velocity
A(4:6, 10:12) = eye(3);         % Angles from angular rates
A(7, 5) = -g;                   % u from theta (gravity coupling)
A(8, 4) = g;                    % v from phi (gravity coupling)

% B matrix (12x4)
B = zeros(12, 4);
B(9, 1) = -1/m;                 % w from thrust
B(10, 2) = 1/Ixx;               % p from roll torque
B(11, 3) = 1/Iyy;               % q from pitch torque
B(12, 4) = 1/Izz;               % r from yaw torque

% C matrix - measured outputs
C = eye(12);

% D matrix - no direct feedthrough
D = zeros(12, 4);
```

### Motor Mixing Matrix

```matlab
% X-configuration motor mixing
% omega^2 = M * [T; tau_phi; tau_theta; tau_psi]

k_f = 1.28e-8;      % Thrust coefficient
k_m = 5.96e-10;     % Moment coefficient

M_inv = [
    1,  1,  1,  1;           % Thrust
    L, -L, -L,  L;           % Roll moment
   -L, -L,  L,  L;           % Pitch moment
   k_m/k_f, -k_m/k_f, k_m/k_f, -k_m/k_f  % Yaw moment
];

M = inv(M_inv);     % Mixing matrix

% Convert control commands to motor speeds
omega_squared = M * [T; tau_phi; tau_theta; tau_psi];
omega = sqrt(max(0, omega_squared));
```

---

## Control Design

### PID Gains

```matlab
% Position control gains
Kp_xy = 1.0;    Ki_xy = 0.0;    Kd_xy = 0.5;
Kp_z = 2.0;     Ki_z = 0.5;     Kd_z = 1.0;

% Attitude control gains
Kp_phi = 6.0;   Ki_phi = 0.0;   Kd_phi = 1.5;
Kp_theta = 6.0; Ki_theta = 0.0; Kd_theta = 1.5;
Kp_psi = 4.0;   Ki_psi = 0.0;   Kd_psi = 1.0;

% Rate control gains
Kp_p = 0.5;     Ki_p = 0.0;     Kd_p = 0.0;
Kp_q = 0.5;     Ki_q = 0.0;     Kd_q = 0.0;
Kp_r = 0.3;     Ki_r = 0.0;     Kd_r = 0.0;
```

### Position Controller

```matlab
function [phi_d, theta_d, T] = position_controller(pos, pos_d, vel, psi)
    % Position error
    e_pos = pos_d - pos;

    % Desired accelerations
    ax_d = Kp_xy * e_pos(1) - Kd_xy * vel(1);
    ay_d = Kp_xy * e_pos(2) - Kd_xy * vel(2);
    az_d = Kp_z * e_pos(3) - Kd_z * vel(3);

    % Thrust
    T = m * (g + az_d);

    % Desired angles (small angle approximation)
    phi_d = (1/g) * (ax_d * sin(psi) - ay_d * cos(psi));
    theta_d = (1/g) * (ax_d * cos(psi) + ay_d * sin(psi));

    % Saturation
    phi_d = max(-0.5, min(0.5, phi_d));
    theta_d = max(-0.5, min(0.5, theta_d));
end
```

---

## Simulink Integration

### Initialization Script

```matlab
TIME_STEP = 8;

% Initialize IMU
imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% Initialize GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% Initialize Gyroscope
gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

% Initialize Camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);

% Initialize Range Sensors
range_front = wb_robot_get_device('range front');
wb_distance_sensor_enable(range_front, TIME_STEP);

% Initialize Motors
motors = cell(1, 4);
motor_names = {'front right propeller', 'rear right propeller', ...
               'rear left propeller', 'front left propeller'};
for i = 1:4
    motors{i} = wb_robot_get_device(motor_names{i});
    wb_motor_set_position(motors{i}, inf);
    wb_motor_set_velocity(motors{i}, 0);
end
```

### Control Loop

```matlab
while wb_robot_step(TIME_STEP) ~= -1
    % Read sensors
    orientation = wb_inertial_unit_get_roll_pitch_yaw(imu);
    position = wb_gps_get_values(gps);
    angular_rates = wb_gyro_get_values(gyro);

    phi = orientation(1);
    theta = orientation(2);
    psi = orientation(3);

    % Position control
    [phi_d, theta_d, T] = position_controller(position, pos_desired, vel, psi);

    % Attitude control
    tau_phi = Kp_phi * (phi_d - phi) - Kd_phi * angular_rates(1);
    tau_theta = Kp_theta * (theta_d - theta) - Kd_theta * angular_rates(2);
    tau_psi = Kp_psi * (psi_d - psi) - Kd_psi * angular_rates(3);

    % Motor mixing
    omega = motor_mixing(T, tau_phi, tau_theta, tau_psi);

    % Apply motor commands
    for i = 1:4
        wb_motor_set_velocity(motors{i}, omega(i));
    end
end
```

---

## Quick Start

1. **Open Webots** and load `examples/mavic_2_pro/worlds/mavic_flight.wbt`
2. **Configure MATLAB** path
3. **Run simulation** for autonomous flight demo
4. **Customize waypoints** in Simulink model

---

## Flight Modes

| Mode | Description |
|------|-------------|
| Hover | Maintain position and altitude |
| Position | GPS-based position control |
| Altitude | Maintain altitude, manual horizontal |
| Attitude | Direct attitude control |

---

## References

- [DJI Mavic 2 Pro](https://www.dji.com/mavic-2)
- [Webots Mavic](https://cyberbotics.com/doc/guide/mavic-2-pro)
- Quan, Q. (2017). "Introduction to Multicopter Design and Control"
- Mahony, R. et al. (2012). "Multirotor Aerial Vehicles"
