% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Tesla Model 3 Autonomous Vehicle Simulink Control
% Author:        Harun Kurt

TIME_STEP = 16;

% Vehicle parameters
wheelbase = 2.875;  % meters
max_steering_angle = 0.5;  % radians
max_speed = 50;  % m/s

% Control gains
Kp_steering = 2.0;
Kd_steering = 0.5;
Kp_speed = 1.0;
Ki_speed = 0.1;

% Target speed (cruise control)
target_speed = 15;  % m/s

% Initialize sensors
% GPS
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

% IMU
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

% Camera
front_camera = wb_robot_get_device('front_camera');
wb_camera_enable(front_camera, TIME_STEP * 2);

% LiDAR
lidar = wb_robot_get_device('lidar');
wb_lidar_enable(lidar, TIME_STEP);
wb_lidar_enable_point_cloud(lidar);

% Radar
radar = wb_robot_get_device('radar');
wb_radar_enable(radar, TIME_STEP);

% Initialize actuators
% Steering
steering = wb_robot_get_device('steering');

% Throttle/Brake
throttle = wb_robot_get_device('throttle');
brake = wb_robot_get_device('brake');

% Wheels
front_left_wheel = wb_robot_get_device('front_left_wheel');
front_right_wheel = wb_robot_get_device('front_right_wheel');
rear_left_wheel = wb_robot_get_device('rear_left_wheel');
rear_right_wheel = wb_robot_get_device('rear_right_wheel');

% Set wheels to velocity control
wb_motor_set_position(front_left_wheel, inf);
wb_motor_set_position(front_right_wheel, inf);
wb_motor_set_position(rear_left_wheel, inf);
wb_motor_set_position(rear_right_wheel, inf);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
