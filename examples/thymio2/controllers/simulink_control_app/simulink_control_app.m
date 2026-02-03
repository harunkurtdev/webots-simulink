% MATLAB controller for Webots
% File:          simulink_control_app.m
% Date:          2024
% Description:   Thymio II Robot Simulink Control
% Author:        Harun Kurt

TIME_STEP = 64;

% Physical parameters
wheel_radius = 0.022;  % meters
axle_length = 0.095;   % meters
max_speed = 9.53;      % rad/s (~14 cm/s linear)

% Behavior parameters
obstacle_threshold = 2000;
line_threshold = 500;

% Initialize front horizontal proximity sensors (5)
prox_front = cell(1, 5);
for i = 0:4
    prox_front{i+1} = wb_robot_get_device(['prox.horizontal.' num2str(i)]);
    wb_distance_sensor_enable(prox_front{i+1}, TIME_STEP);
end

% Initialize rear horizontal proximity sensors (2)
prox_rear = cell(1, 2);
for i = 5:6
    prox_rear{i-4} = wb_robot_get_device(['prox.horizontal.' num2str(i)]);
    wb_distance_sensor_enable(prox_rear{i-4}, TIME_STEP);
end

% Initialize ground sensors (2)
prox_ground = cell(1, 2);
for i = 0:1
    prox_ground{i+1} = wb_robot_get_device(['prox.ground.' num2str(i)]);
    wb_distance_sensor_enable(prox_ground{i+1}, TIME_STEP);
end

% Initialize accelerometer
accelerometer = wb_robot_get_device('acc');
wb_accelerometer_enable(accelerometer, TIME_STEP);

% Initialize motors
left_motor = wb_robot_get_device('motor.left');
right_motor = wb_robot_get_device('motor.right');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% Initialize LEDs
leds_top = wb_robot_get_device('leds.top');
leds_bottom_left = wb_robot_get_device('leds.bottom.left');
leds_bottom_right = wb_robot_get_device('leds.bottom.right');

% Load Simulink model
open_system('simulink_control');
load_system('simulink_control');
