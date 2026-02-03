% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Khepera IV Robot Simulink Control
% Author:        Harun Kurt

TIME_STEP = 16;

% Physical parameters
wheel_radius = 0.02175;  % meters
axle_length = 0.1054;    % meters
max_speed = 47.6;        % rad/s

% Control gains
Kp = 1.0;
Ki = 0.1;
Kd = 0.05;

% Initialize proximity sensors (8 IR sensors)
prox_sensors = cell(1, 8);
for i = 0:7
    prox_sensors{i+1} = wb_robot_get_device(['proximity sensor ' num2str(i)]);
    wb_distance_sensor_enable(prox_sensors{i+1}, TIME_STEP);
end

% Initialize ultrasonic sensors
us_sensors = cell(1, 5);
for i = 0:4
    us_sensors{i+1} = wb_robot_get_device(['ultrasonic sensor ' num2str(i)]);
    wb_distance_sensor_enable(us_sensors{i+1}, TIME_STEP);
end

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

% Initialize camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP * 2);

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
