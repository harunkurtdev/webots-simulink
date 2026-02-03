% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Spot Quadruped Robot Simulink Control
% Author:        Harun Kurt

TIME_STEP = 8;

% Gait parameters
gait_period = 0.4;  % seconds
step_height = 0.06; % meters
stride_length = 0.1; % meters

% Physical parameters
body_height = 0.5;  % meters
leg_length = 0.4;   % meters

% Initialize IMU
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% Initialize leg motors - Front Left
fl_shoulder_abd = wb_robot_get_device('front left shoulder abduction motor');
fl_shoulder_rot = wb_robot_get_device('front left shoulder rotation motor');
fl_elbow = wb_robot_get_device('front left elbow motor');

% Front Right
fr_shoulder_abd = wb_robot_get_device('front right shoulder abduction motor');
fr_shoulder_rot = wb_robot_get_device('front right shoulder rotation motor');
fr_elbow = wb_robot_get_device('front right elbow motor');

% Rear Left
rl_shoulder_abd = wb_robot_get_device('rear left shoulder abduction motor');
rl_shoulder_rot = wb_robot_get_device('rear left shoulder rotation motor');
rl_elbow = wb_robot_get_device('rear left elbow motor');

% Rear Right
rr_shoulder_abd = wb_robot_get_device('rear right shoulder abduction motor');
rr_shoulder_rot = wb_robot_get_device('rear right shoulder rotation motor');
rr_elbow = wb_robot_get_device('rear right elbow motor');

% Set initial standing pose
standing_angle = -0.5;  % radians
wb_motor_set_position(fl_shoulder_rot, standing_angle);
wb_motor_set_position(fr_shoulder_rot, standing_angle);
wb_motor_set_position(rl_shoulder_rot, standing_angle);
wb_motor_set_position(rr_shoulder_rot, standing_angle);

wb_motor_set_position(fl_elbow, standing_angle * 2);
wb_motor_set_position(fr_elbow, standing_angle * 2);
wb_motor_set_position(rl_elbow, standing_angle * 2);
wb_motor_set_position(rr_elbow, standing_angle * 2);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
