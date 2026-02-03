% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Blimp (Airship) Simulink Control
% Author:        Harun Kurt
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

TIME_STEP = 16;

% Physical parameters
mass = 2.0;           % Total mass (kg)
V_envelope = 1.5;     % Envelope volume (m³)
rho_air = 1.225;      % Air density (kg/m³)
g = 9.81;             % Gravity (m/s²)

% Buoyancy calculation
F_buoyancy = rho_air * V_envelope * g;
F_gravity = mass * g;
net_lift = F_buoyancy - F_gravity;

% Control gains
Kp_z = 5.0; Ki_z = 0.5; Kd_z = 2.0;
Kp_yaw = 2.0; Kd_yaw = 0.5;
Kp_xy = 1.0; Kd_xy = 0.5;

% Target altitude
target_altitude = 5.0;  % meters

% Initialize sensors
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);

inertial_unit = wb_robot_get_device('inertial unit');
wb_inertial_unit_enable(inertial_unit, TIME_STEP);

% Initialize altimeter (distance sensor)
altimeter = wb_robot_get_device('altimeter');
wb_distance_sensor_enable(altimeter, TIME_STEP);

% Initialize propulsion motors
left_motor = wb_robot_get_device('left_motor');
right_motor = wb_robot_get_device('right_motor');
tail_motor = wb_robot_get_device('tail_motor');

% Set motors to velocity control mode
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_position(tail_motor, inf);

wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);
wb_motor_set_velocity(tail_motor, 0);

% Initialize control surfaces (if present)
% elevator = wb_robot_get_device('elevator');
% rudder = wb_robot_get_device('rudder');

% Initialize camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP * 4);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
