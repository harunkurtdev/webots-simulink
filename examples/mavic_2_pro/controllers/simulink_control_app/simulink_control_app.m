% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Mavic 2 Pro Drone Simulink Control
% Author:        Harun Kurt
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 8;  % 125 Hz control rate

% Physical parameters
m = 0.907;      % Mass in kg
g = 9.81;       % Gravity m/s^2
Ixx = 0.01;     % Moment of inertia
Iyy = 0.01;
Izz = 0.02;

% Control gains
Kp_z = 2.0; Ki_z = 0.5; Kd_z = 1.0;
Kp_xy = 1.0; Ki_xy = 0.1; Kd_xy = 0.5;
Kp_att = 5.0; Kd_att = 1.0;
Kp_yaw = 2.0;

% Initialize IMU sensors
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

imu = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(imu, TIME_STEP);

% Initialize GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% Initialize compass
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

% Initialize range sensors
front_range = wb_robot_get_device('front range');
wb_distance_sensor_enable(front_range, TIME_STEP);

% Initialize motors (X configuration)
front_right_motor = wb_robot_get_device('front right propeller');
front_left_motor = wb_robot_get_device('front left propeller');
rear_left_motor = wb_robot_get_device('rear left propeller');
rear_right_motor = wb_robot_get_device('rear right propeller');

% Set motors to velocity control mode
wb_motor_set_position(front_right_motor, inf);
wb_motor_set_position(front_left_motor, inf);
wb_motor_set_position(rear_left_motor, inf);
wb_motor_set_position(rear_right_motor, inf);

% Initialize with zero velocity
wb_motor_set_velocity(front_right_motor, 0);
wb_motor_set_velocity(front_left_motor, 0);
wb_motor_set_velocity(rear_left_motor, 0);
wb_motor_set_velocity(rear_right_motor, 0);

% Initialize camera gimbal
camera_pitch_motor = wb_robot_get_device('camera pitch');
camera_roll_motor = wb_robot_get_device('camera roll');

% Initialize camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP * 4);

% Target altitude for hover
target_altitude = 2.0;  % meters

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
