% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Firebird 6 Robot Simulink Control
% Author:        Harun Kurt

TIME_STEP = 16;

% Physical parameters
wheel_radius = 0.025;  % meters
axle_length = 0.12;    % meters
max_speed = 20;        % rad/s

% Control gains
Kp = 2.0;
Ki = 0.5;
Kd = 0.1;

% Initialize IR proximity sensors (8)
ps = cell(1, 8);
for i = 0:7
    ps{i+1} = wb_robot_get_device(['ps' num2str(i)]);
    wb_distance_sensor_enable(ps{i+1}, TIME_STEP);
end

% Initialize Sharp distance sensors
sharp_front = wb_robot_get_device('sharp front');
sharp_left = wb_robot_get_device('sharp left');
sharp_right = wb_robot_get_device('sharp right');
wb_distance_sensor_enable(sharp_front, TIME_STEP);
wb_distance_sensor_enable(sharp_left, TIME_STEP);
wb_distance_sensor_enable(sharp_right, TIME_STEP);

% Initialize IMU
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP);

gyro = wb_robot_get_device('gyro');
wb_gyro_enable(gyro, TIME_STEP);

% Initialize motors
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% Initialize encoders
left_encoder = wb_robot_get_device('left wheel sensor');
right_encoder = wb_robot_get_device('right wheel sensor');
wb_position_sensor_enable(left_encoder, TIME_STEP);
wb_position_sensor_enable(right_encoder, TIME_STEP);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
