# NAO Humanoid Robot

This example demonstrates bipedal humanoid robot control using the SoftBank Robotics NAO robot with MATLAB/Simulink. NAO is a 58cm tall humanoid robot designed for education, research, and entertainment.

---

## Overview

| Property | Value |
|----------|-------|
| **Type** | Bipedal Humanoid Robot |
| **Robot** | SoftBank NAO V6 |
| **Difficulty** | Advanced |
| **Control Method** | Joint Control / Gait Planning |
| **DOF** | 25 (full body) |

---

## Features

- **Bipedal Walking**: Dynamic gait generation
- **Upper Body Control**: Arm gestures and manipulation
- **Multi-Modal Sensing**: Cameras, microphones, touch sensors
- **Balance Control**: IMU-based stabilization
- **Speech**: Text-to-speech and recognition
- **LED Feedback**: Eye and body LEDs

---

## Project Structure

```
nao_humanoid/
├── controllers/
│   └── simulink_control_app/
│       ├── simulink_control_app.m        # Main MATLAB controller
│       ├── simulink_control.slx          # Simulink control model
│       ├── gait_generator.m              # Walking pattern generation
│       ├── balance_controller.m          # ZMP-based balance
│       ├── wb_motor_set_position.m       # Joint position control
│       ├── wb_accelerometer_get_values.m # IMU reading
│       ├── wb_gyro_get_values.m          # Gyroscope reading
│       ├── wb_camera_get_image.m         # Vision
│       └── wb_robot_step.m               # Simulation step
└── worlds/
    └── nao_walking.wbt                   # Webots world file
```

---

## Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Height | 574 mm |
| Weight | 5.48 kg |
| Battery | Li-Ion (48.6 Wh) |
| Autonomy | ~90 min active |

### Joint Configuration

| Body Part | Joints | DOF |
|-----------|--------|-----|
| Head | HeadYaw, HeadPitch | 2 |
| Left Arm | LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw | 5 |
| Right Arm | RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw | 5 |
| Left Leg | LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll | 6 |
| Right Leg | RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll | 6 |
| Hands | LHand, RHand | 2 (gripper) |

### Sensors

| Sensor | Quantity | Purpose |
|--------|----------|---------|
| Cameras | 2 (head) | Vision |
| Microphones | 4 | Sound localization |
| Sonar | 2 | Distance measurement |
| IMU | 1 | Balance |
| FSR | 8 (4/foot) | Ground contact |
| Touch | 14 | Interaction |
| Joint Encoders | 25 | Position feedback |

---

## Walking Control

### Zero Moment Point (ZMP) Based Gait

```matlab
function [q_left, q_right] = gait_generator(t, step_length, step_height, period)
    % Phase calculation
    phase = mod(t, period) / period;

    % Foot trajectories
    if phase < 0.5
        % Left foot swing, right foot stance
        [x_l, z_l] = swing_trajectory(phase * 2, step_length, step_height);
        [x_r, z_r] = stance_trajectory();
    else
        % Right foot swing, left foot stance
        [x_r, z_r] = swing_trajectory((phase - 0.5) * 2, step_length, step_height);
        [x_l, z_l] = stance_trajectory();
    end

    % Inverse kinematics for each leg
    q_left = leg_ik(x_l, z_l);
    q_right = leg_ik(x_r, z_r);
end

function [x, z] = swing_trajectory(s, length, height)
    % Cycloid-based swing foot trajectory
    x = length * (s - sin(2*pi*s) / (2*pi));
    z = height * (1 - cos(2*pi*s)) / 2;
end
```

### Balance Controller

```matlab
function torque_adj = balance_controller(imu_data, fsr_data)
    % Read IMU
    roll = imu_data(1);
    pitch = imu_data(2);

    % PD control for balance
    Kp_roll = 10; Kd_roll = 2;
    Kp_pitch = 15; Kd_pitch = 3;

    torque_roll = Kp_roll * roll + Kd_roll * roll_dot;
    torque_pitch = Kp_pitch * pitch + Kd_pitch * pitch_dot;

    % Apply to ankle joints
    torque_adj = [torque_roll, torque_pitch];
end
```

---

## Controller Implementation

### Initialization

```matlab
% simulink_control_app.m
TIME_STEP = 16;

% Head joints
HeadYaw = wb_robot_get_device('HeadYaw');
HeadPitch = wb_robot_get_device('HeadPitch');

% Left arm joints
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
LShoulderRoll = wb_robot_get_device('LShoulderRoll');
LElbowYaw = wb_robot_get_device('LElbowYaw');
LElbowRoll = wb_robot_get_device('LElbowRoll');

% Right arm joints
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
RShoulderRoll = wb_robot_get_device('RShoulderRoll');
RElbowYaw = wb_robot_get_device('RElbowYaw');
RElbowRoll = wb_robot_get_device('RElbowRoll');

% Left leg joints
LHipYawPitch = wb_robot_get_device('LHipYawPitch');
LHipRoll = wb_robot_get_device('LHipRoll');
LHipPitch = wb_robot_get_device('LHipPitch');
LKneePitch = wb_robot_get_device('LKneePitch');
LAnklePitch = wb_robot_get_device('LAnklePitch');
LAnkleRoll = wb_robot_get_device('LAnkleRoll');

% Right leg joints
RHipYawPitch = wb_robot_get_device('RHipYawPitch');
RHipRoll = wb_robot_get_device('RHipRoll');
RHipPitch = wb_robot_get_device('RHipPitch');
RKneePitch = wb_robot_get_device('RKneePitch');
RAnklePitch = wb_robot_get_device('RAnklePitch');
RAnkleRoll = wb_robot_get_device('RAnkleRoll');

% IMU sensors
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

% Cameras
CameraTop = wb_robot_get_device('CameraTop');
wb_camera_enable(CameraTop, TIME_STEP * 2);

% Initialize standing pose
standing_pose();

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
```

---

## Quick Start

1. **Open Webots** and load `examples/nao_humanoid/worlds/nao_walking.wbt`

2. **Configure MATLAB** path for Webots

3. **Run simulation**:
   - NAO initializes in standing pose
   - Balance controller activates
   - Walking sequence begins

4. **Customize behavior**:
   - Modify gait parameters
   - Add arm gestures
   - Implement vision-based navigation

---

## Applications

- **Research**: Bipedal locomotion, human-robot interaction
- **Education**: Robotics and AI concepts
- **Entertainment**: Dance, sports, games
- **Healthcare**: Therapy assistance

---

## References

- [SoftBank Robotics NAO](https://www.softbankrobotics.com/emea/en/nao)
- [Webots NAO Documentation](https://cyberbotics.com/doc/guide/nao)
- Vukobratović, M. (2004). "Zero-Moment Point - Thirty Five Years of Its Life"
- Kajita, S. (2003). "Biped Walking Pattern Generation"
