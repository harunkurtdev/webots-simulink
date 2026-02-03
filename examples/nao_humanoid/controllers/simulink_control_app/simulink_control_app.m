% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   NAO Humanoid Robot Simulink Control
% Author:        Harun Kurt
% Modifications:

TIME_STEP = 16;

% Gait parameters
step_length = 0.04;  % meters
step_height = 0.02;  % meters
step_period = 0.5;   % seconds

% Balance control gains
Kp_pitch = 15.0;
Kd_pitch = 3.0;
Kp_roll = 10.0;
Kd_roll = 2.0;

% Head joints
HeadYaw = wb_robot_get_device('HeadYaw');
HeadPitch = wb_robot_get_device('HeadPitch');

% Left arm joints
LShoulderPitch = wb_robot_get_device('LShoulderPitch');
LShoulderRoll = wb_robot_get_device('LShoulderRoll');
LElbowYaw = wb_robot_get_device('LElbowYaw');
LElbowRoll = wb_robot_get_device('LElbowRoll');
LWristYaw = wb_robot_get_device('LWristYaw');

% Right arm joints
RShoulderPitch = wb_robot_get_device('RShoulderPitch');
RShoulderRoll = wb_robot_get_device('RShoulderRoll');
RElbowYaw = wb_robot_get_device('RElbowYaw');
RElbowRoll = wb_robot_get_device('RElbowRoll');
RWristYaw = wb_robot_get_device('RWristYaw');

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

inertial_unit = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(inertial_unit, TIME_STEP);

% Top camera
CameraTop = wb_robot_get_device('CameraTop');
wb_camera_enable(CameraTop, TIME_STEP * 2);

% Set initial standing pose
wb_motor_set_position(LShoulderPitch, 1.4);
wb_motor_set_position(RShoulderPitch, 1.4);
wb_motor_set_position(LShoulderRoll, 0.2);
wb_motor_set_position(RShoulderRoll, -0.2);
wb_motor_set_position(LElbowRoll, -0.5);
wb_motor_set_position(RElbowRoll, 0.5);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
